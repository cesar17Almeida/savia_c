#include "savia/ble_codec.h"
#include "savia/cbor.h"
#include "savia/protocol.h"
#include "savia/pinmap.h"
#include <string.h>

static const char *kind_str(uint8_t kind) {
    switch (kind) {
        case READING_SOIL_MOISTURE:    return "soil_moisture";
        case READING_SOIL_TEMPERATURE: return "soil_temperature";
        case READING_AIR_TEMPERATURE:  return "air_temperature";
        default:                       return "unknown";
    }
}

// Inverse of kind_str: reading-kind token -> enum. Unknown -> -1 (point dropped).
static int kind_from_str(const char *s, size_t n) {
    if (cbor_text_eq(s, n, "soil_moisture"))    return READING_SOIL_MOISTURE;
    if (cbor_text_eq(s, n, "soil_temperature")) return READING_SOIL_TEMPERATURE;
    if (cbor_text_eq(s, n, "air_temperature"))  return READING_AIR_TEMPERATURE;
    return -1;
}

size_t ble_serialize_readings(const savia_reading_t *rows, size_t n,
                              uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_array(&w, n);
    for (size_t i = 0; i < n; i++) {
        cbor_w_map(&w, 5);
        cbor_w_textz(&w, "ts_ms");    cbor_w_uint(&w, rows[i].ts_ms);
        cbor_w_textz(&w, "port");     cbor_w_uint(&w, rows[i].port);
        cbor_w_textz(&w, "kind");     cbor_w_textz(&w, kind_str(rows[i].kind));
        cbor_w_textz(&w, "value");    cbor_w_double(&w, (double) rows[i].value);
        cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, rows[i].depth_cm);
    }
    return w.overflow ? 0 : w.len;
}

size_t ble_chunk_total(size_t payload_len, size_t chunk_size) {
    if (chunk_size == 0) chunk_size = BLE_DATA_CHUNK_BYTES;
    size_t t = (payload_len + chunk_size - 1) / chunk_size;
    return t == 0 ? 1 : t;   // empty payload -> single eof frame
}

size_t ble_chunk_make_frame(const uint8_t *payload, size_t len, size_t chunk_size,
                            size_t seq, uint8_t *out, size_t cap) {
    if (chunk_size == 0) chunk_size = BLE_DATA_CHUNK_BYTES;
    size_t total = ble_chunk_total(len, chunk_size);
    size_t start = seq * chunk_size;
    size_t end = start + chunk_size;
    if (end > len) end = len;
    size_t clen = (start < len) ? (end - start) : 0;

    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 6);
    cbor_w_textz(&w, "v");   cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
    cbor_w_textz(&w, "op");  cbor_w_textz(&w, SAVIA_OP_CHUNK);
    cbor_w_textz(&w, "s");   cbor_w_uint(&w, seq);
    cbor_w_textz(&w, "t");   cbor_w_uint(&w, total);
    cbor_w_textz(&w, "eof"); cbor_w_bool(&w, seq == total - 1);
    cbor_w_textz(&w, "p");   cbor_w_bytes(&w, payload + start, clen);
    return w.overflow ? 0 : w.len;
}

void ble_chunk_encode(const uint8_t *payload, size_t len, size_t chunk_size,
                      ble_frame_emit_fn emit, void *ctx) {
    size_t total = ble_chunk_total(len, chunk_size);
    uint8_t frame[BLE_DATA_CHUNK_BYTES + 64];
    for (size_t seq = 0; seq < total; seq++) {
        size_t fl = ble_chunk_make_frame(payload, len, chunk_size, seq, frame, sizeof(frame));
        if (fl) emit(frame, fl, ctx);
    }
}

bool ble_parse_data_request(const uint8_t *buf, size_t len, ble_data_request_t *out) {
    memset(out, 0, sizeof(*out));
    cbor_reader_t r;
    cbor_r_init(&r, buf, len);

    uint64_t count;
    if (!cbor_r_map(&r, &count)) return false;
    for (uint64_t i = 0; ; i++) {
        if (count == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
        else if (i >= count) break;
        const char *k; size_t kn;
        if (!cbor_r_text(&r, &k, &kn)) return false;

        if (cbor_text_eq(k, kn, "v")) {
            uint64_t v; if (!cbor_r_uint(&r, &v)) return false;
            out->version = (int) v;
        } else if (cbor_text_eq(k, kn, "op")) {
            const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
            if (sn >= sizeof(out->op)) sn = sizeof(out->op) - 1;
            memcpy(out->op, s, sn); out->op[sn] = 0;
        } else if (cbor_text_eq(k, kn, "kind")) {
            const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
            if (sn >= sizeof(out->kind)) sn = sizeof(out->kind) - 1;
            memcpy(out->kind, s, sn); out->kind[sn] = 0;
        } else if (cbor_text_eq(k, kn, "from")) {
            if (!cbor_r_null(&r)) { if (!cbor_r_uint(&r, &out->from_ms)) return false; out->has_from = true; }
        } else if (cbor_text_eq(k, kn, "to")) {
            if (!cbor_r_null(&r)) { if (!cbor_r_uint(&r, &out->to_ms)) return false; out->has_to = true; }
        } else if (cbor_text_eq(k, kn, "limit")) {
            if (!cbor_r_null(&r)) { if (!cbor_r_uint(&r, &out->limit)) return false; out->has_limit = true; }
        } else if (cbor_text_eq(k, kn, "cmd")) {
            if (!cbor_r_null(&r)) {
                const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
                if (sn >= sizeof(out->cmd)) sn = sizeof(out->cmd) - 1;
                memcpy(out->cmd, s, sn); out->cmd[sn] = 0; out->has_cmd = true;
            }
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    out->ok = !r.err;
    return out->ok;
}

size_t ble_serialize_at_result(const lora_at_result_t *r, uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 3);
    cbor_w_textz(&w, "seq"); cbor_w_uint(&w, r->seq);
    cbor_w_textz(&w, "cmd"); cbor_w_textz(&w, r->cmd);
    cbor_w_textz(&w, "lines");
    uint8_t n = r->count <= LORA_AT_MAX_LINES ? r->count : LORA_AT_MAX_LINES;
    cbor_w_array(&w, n);
    for (uint8_t i = 0; i < n; i++) cbor_w_textz(&w, r->lines[i]);
    return w.overflow ? 0 : w.len;
}

// Parse one {ts_ms, kind, value, depth_cm?, port?} map into *rd. Returns true if
// the point is complete (ts_ms + known kind + value present); a malformed/unknown
// point still consumes its bytes but yields false so the caller can skip it.
static bool parse_ingest_point(cbor_reader_t *r, savia_reading_t *rd) {
    memset(rd, 0, sizeof(*rd));
    rd->port = 1;                            // default logical port
    int kind = -1;
    bool has_ts = false, has_value = false;
    uint64_t fcount;
    if (!cbor_r_map(r, &fcount)) return false;
    for (uint64_t f = 0; ; f++) {
        if (fcount == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(r)) break; }
        else if (f >= fcount) break;
        const char *fk; size_t fkn;
        if (!cbor_r_text(r, &fk, &fkn)) return false;
        if (cbor_text_eq(fk, fkn, "ts_ms")) {
            uint64_t v; if (!cbor_r_uint(r, &v)) return false; rd->ts_ms = v; has_ts = true;
        } else if (cbor_text_eq(fk, fkn, "kind")) {
            const char *s; size_t sn; if (!cbor_r_text(r, &s, &sn)) return false;
            kind = kind_from_str(s, sn);
        } else if (cbor_text_eq(fk, fkn, "value")) {
            double d; if (!cbor_r_double(r, &d)) return false; rd->value = (float) d; has_value = true;
        } else if (cbor_text_eq(fk, fkn, "depth_cm")) {
            if (!cbor_r_null(r)) { uint64_t v; if (!cbor_r_uint(r, &v)) return false; rd->depth_cm = (uint8_t) v; }
        } else if (cbor_text_eq(fk, fkn, "port")) {
            if (!cbor_r_null(r)) { uint64_t v; if (!cbor_r_uint(r, &v)) return false; rd->port = (uint8_t) v; }
        } else {
            if (!cbor_r_skip(r)) return false;
        }
    }
    if (kind < 0 || !has_ts || !has_value) return false;
    rd->kind = (uint8_t) kind;
    return true;
}

size_t ble_parse_ingest(const uint8_t *buf, size_t len,
                        savia_reading_t *out, size_t cap, bool *ok) {
    *ok = false;
    cbor_reader_t r;
    cbor_r_init(&r, buf, len);
    uint64_t count;
    if (!cbor_r_map(&r, &count)) return 0;
    size_t n = 0;
    int ver = 0;
    bool got_data = false;
    for (uint64_t i = 0; ; i++) {
        if (count == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
        else if (i >= count) break;
        const char *k; size_t kn;
        if (!cbor_r_text(&r, &k, &kn)) return 0;
        if (cbor_text_eq(k, kn, "v")) {
            uint64_t v; if (!cbor_r_uint(&r, &v)) return 0; ver = (int) v;
        } else if (cbor_text_eq(k, kn, "data")) {
            uint64_t acount;
            if (!cbor_r_array(&r, &acount)) return 0;
            for (uint64_t j = 0; ; j++) {
                if (acount == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
                else if (j >= acount) break;
                savia_reading_t rd;
                bool good = parse_ingest_point(&r, &rd);
                if (r.err) return 0;
                if (good && n < cap) out[n++] = rd;   // drop malformed/unknown points
            }
            got_data = true;
        } else {
            if (!cbor_r_skip(&r)) return 0;   // skips "op" and any unknown key
        }
    }
    *ok = !r.err && got_data && ver == SAVIA_PROTOCOL_VERSION;
    return n;
}

size_t ble_serialize_ingest_ok(size_t created, size_t updated, uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 4);
    cbor_w_textz(&w, "v");       cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
    cbor_w_textz(&w, "op");      cbor_w_textz(&w, SAVIA_OP_INGEST_OK);
    cbor_w_textz(&w, "created"); cbor_w_uint(&w, created);
    cbor_w_textz(&w, "updated"); cbor_w_uint(&w, updated);
    return w.overflow ? 0 : w.len;
}

size_t ble_serialize_aggregations(const savia_aggregate_t *rows, size_t n,
                                  uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_array(&w, n);
    for (size_t i = 0; i < n; i++) {
        cbor_w_map(&w, 8);
        cbor_w_textz(&w, "hour_ms");  cbor_w_uint(&w, rows[i].hour_ms);
        cbor_w_textz(&w, "port");     cbor_w_uint(&w, rows[i].port);
        cbor_w_textz(&w, "kind");     cbor_w_textz(&w, kind_str(rows[i].kind));
        cbor_w_textz(&w, "count");    cbor_w_uint(&w, rows[i].count);
        cbor_w_textz(&w, "mean");     cbor_w_double(&w, (double) rows[i].mean);
        cbor_w_textz(&w, "min");      cbor_w_double(&w, (double) rows[i].min);
        cbor_w_textz(&w, "max");      cbor_w_double(&w, (double) rows[i].max);
        cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, rows[i].depth_cm);
    }
    return w.overflow ? 0 : w.len;
}

size_t ble_serialize_predictions(const savia_prediction_t *rows, size_t n,
                                 uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_array(&w, n);
    for (size_t i = 0; i < n; i++) {
        cbor_w_map(&w, 6);
        cbor_w_textz(&w, "ts_ms"); cbor_w_uint(&w, rows[i].ts_ms);
        cbor_w_textz(&w, "model"); cbor_w_textz(&w, rows[i].model);
        cbor_w_textz(&w, "kind");  cbor_w_textz(&w, rows[i].kind);
        cbor_w_textz(&w, "port");
        if (rows[i].has_port) cbor_w_uint(&w, rows[i].port); else cbor_w_null(&w);
        cbor_w_textz(&w, "value"); cbor_w_double(&w, (double) rows[i].value);
        cbor_w_textz(&w, "confidence");
        if (rows[i].has_confidence) cbor_w_double(&w, (double) rows[i].confidence);
        else cbor_w_null(&w);
    }
    return w.overflow ? 0 : w.len;
}

size_t ble_serialize_status(uint32_t uptime_s, uint64_t last_sync_ms,
                            uint64_t weather_updated_ms, const lora_status_t *lora,
                            uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 6);
    cbor_w_textz(&w, "v");        cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
    cbor_w_textz(&w, "fw");       cbor_w_textz(&w, SAVIA_FW_VERSION);
    cbor_w_textz(&w, "uptime_s"); cbor_w_uint(&w, uptime_s);
    cbor_w_textz(&w, "last_sync_ms");
    if (last_sync_ms) cbor_w_uint(&w, last_sync_ms); else cbor_w_null(&w);
    cbor_w_textz(&w, "weather_updated_ms");
    if (weather_updated_ms) cbor_w_uint(&w, weather_updated_ms); else cbor_w_null(&w);

    // LoRa link: joined proves a TTN gateway relayed the OTAA handshake; rssi/snr
    // are the downlink signal from the last uplink ACK (null until first measured).
    bool sig = lora && lora->has_signal;
    cbor_w_textz(&w, "lora");
    cbor_w_map(&w, 7);
    cbor_w_textz(&w, "inited"); cbor_w_bool(&w, lora && lora->inited);   // module replied to AT
    cbor_w_textz(&w, "joined"); cbor_w_bool(&w, lora && lora->joined);
    cbor_w_textz(&w, "rssi");   if (sig) cbor_w_int(&w, lora->rssi_dbm); else cbor_w_null(&w);
    cbor_w_textz(&w, "snr");    if (sig) cbor_w_double(&w, lora->snr_ddb / 10.0); else cbor_w_null(&w);
    cbor_w_textz(&w, "last_ms");
    if (lora && lora->last_signal_ms) cbor_w_uint(&w, lora->last_signal_ms); else cbor_w_null(&w);
    cbor_w_textz(&w, "module"); cbor_w_textz(&w, lora ? lora->module : "");  // AT+VER reply
    cbor_w_textz(&w, "seq");    cbor_w_uint(&w, lora ? lora->seq : 0);

    return w.overflow ? 0 : w.len;
}

size_t ble_serialize_count(uint64_t count, uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 1);
    cbor_w_textz(&w, "count"); cbor_w_uint(&w, count);
    return w.overflow ? 0 : w.len;
}

bool ble_parse_time_sync(const uint8_t *buf, size_t len, uint64_t *ms_out) {
    cbor_reader_t r;
    cbor_r_init(&r, buf, len);
    uint64_t count;
    if (!cbor_r_map(&r, &count)) return false;
    bool ok_op = false, have_ms = false;
    int ver = 0;
    for (uint64_t i = 0; ; i++) {
        if (count == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
        else if (i >= count) break;
        const char *k; size_t kn;
        if (!cbor_r_text(&r, &k, &kn)) return false;
        if (cbor_text_eq(k, kn, "v")) {
            uint64_t v; if (!cbor_r_uint(&r, &v)) return false; ver = (int) v;
        } else if (cbor_text_eq(k, kn, "op")) {
            const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
            ok_op = cbor_text_eq(s, sn, "set");
        } else if (cbor_text_eq(k, kn, "ms")) {
            if (!cbor_r_uint(&r, ms_out)) return false; have_ms = true;
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    return !r.err && ok_op && have_ms && ver == SAVIA_PROTOCOL_VERSION;
}

bool ble_parse_weather(const uint8_t *buf, size_t len) {
    cbor_reader_t r;
    cbor_r_init(&r, buf, len);
    uint64_t count;
    if (!cbor_r_map(&r, &count)) return false;
    bool ok_op = false, have_data = false;
    int ver = 0;
    for (uint64_t i = 0; ; i++) {
        if (count == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
        else if (i >= count) break;
        const char *k; size_t kn;
        if (!cbor_r_text(&r, &k, &kn)) return false;
        if (cbor_text_eq(k, kn, "v")) {
            uint64_t v; if (!cbor_r_uint(&r, &v)) return false; ver = (int) v;
        } else if (cbor_text_eq(k, kn, "op")) {
            const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
            ok_op = cbor_text_eq(s, sn, "upd");
        } else if (cbor_text_eq(k, kn, "data")) {
            if (!cbor_r_skip(&r)) return false; have_data = true;   // contents unused on WH
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    return !r.err && ok_op && have_data && ver == SAVIA_PROTOCOL_VERSION;
}

// --- config characteristic (0013) -------------------------------------------

static const char *sensor_type_str(savia_sensor_type_t t) {
    switch (t) {
        case SENSOR_SDI12_AQUACHECK:  return "sdi12_aquacheck";
        case SENSOR_SDI12_GENERIC:    return "sdi12_generic";
        case SENSOR_ANALOG_LINEAR:    return "analog_linear";
        case SENSOR_ONEWIRE_DS18B20:  return "onewire_ds18b20";
        default:                      return "none";
    }
}

static savia_sensor_type_t sensor_type_from_str(const char *s, size_t n) {
    if (cbor_text_eq(s, n, "sdi12_aquacheck")) return SENSOR_SDI12_AQUACHECK;
    if (cbor_text_eq(s, n, "sdi12_generic"))   return SENSOR_SDI12_GENERIC;
    if (cbor_text_eq(s, n, "analog_linear"))   return SENSOR_ANALOG_LINEAR;
    if (cbor_text_eq(s, n, "onewire_ds18b20")) return SENSOR_ONEWIRE_DS18B20;
    return SENSOR_NONE;
}

size_t ble_serialize_config(const savia_device_id_t *dev,
                            const station_config_t *cfg,
                            uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 11);

    cbor_w_textz(&w, "v"); cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);

    // device: static identity (uptime/last_sync are in the status characteristic).
    cbor_w_textz(&w, "device");
    cbor_w_map(&w, 3);
    cbor_w_textz(&w, "model"); cbor_w_textz(&w, dev->model);
    cbor_w_textz(&w, "mcu");   cbor_w_textz(&w, dev->mcu);
    cbor_w_textz(&w, "fw");    cbor_w_textz(&w, dev->fw);

    cbor_w_textz(&w, "name");       cbor_w_textz(&w, cfg->ble_name);
    cbor_w_textz(&w, "sleep_s");    cbor_w_uint(&w, cfg->sleep_seconds);
    cbor_w_textz(&w, "deep_sleep"); cbor_w_bool(&w, cfg->deep_sleep_enabled);
    cbor_w_textz(&w, "capture_s");  cbor_w_uint(&w, cfg->capture_interval_s);
    cbor_w_textz(&w, "daily_hour"); cbor_w_uint(&w, cfg->daily_hour);
    cbor_w_textz(&w, "mock");       cbor_w_bool(&w, cfg->mock_enabled);
    cbor_w_textz(&w, "log_level");  cbor_w_uint(&w, cfg->log_level);
    cbor_w_textz(&w, "wake_gpio");  cbor_w_uint(&w, cfg->wake_button_gpio);

    // sensors: one entry per configured slot. AquaCheck stays {port,gpio,type,addr}
    // (fixed layout); the other types also carry the installer-supplied decoding so
    // the app can show it and re-send it unchanged: analog -> kind/depth/scale/offset,
    // 1-Wire -> kind/depth, SDI-12 generic -> chan[] of {kind,depth}.
    uint8_t nsens = cfg->sensor_count <= SAVIA_MAX_SENSORS ? cfg->sensor_count : SAVIA_MAX_SENSORS;
    cbor_w_textz(&w, "sensors");
    cbor_w_array(&w, nsens);
    for (uint8_t i = 0; i < nsens; i++) {
        const savia_sensor_slot_t *s = &cfg->sensors[i];
        bool is_analog  = s->type == SENSOR_ANALOG_LINEAR;
        bool is_1wire   = s->type == SENSOR_ONEWIRE_DS18B20;
        bool is_generic = s->type == SENSOR_SDI12_GENERIC;
        uint8_t fields = 5;                          // port, gpio, type, addr, interval_s
        if (is_analog || is_1wire) fields += 2;      // kind, depth_cm
        if (is_analog)  fields += 2;                 // scale, offset
        if (is_generic) fields += 1;                 // chan

        cbor_w_map(&w, fields);
        cbor_w_textz(&w, "port"); cbor_w_uint(&w, (uint64_t)(i + 1));
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, s->gpio);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, sensor_type_str(s->type));
        char addr[2] = { s->address, 0 };
        cbor_w_textz(&w, "addr"); cbor_w_textz(&w, addr);
        // per-sensor cadence (0 = follow the global capture_s); always sent so the app can show/edit it.
        cbor_w_textz(&w, "interval_s"); cbor_w_uint(&w, s->sample_interval_s);
        if (is_analog || is_1wire) {
            cbor_w_textz(&w, "kind");     cbor_w_textz(&w, kind_str(s->kind));
            cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, s->depth_cm);
        }
        if (is_analog) {
            cbor_w_textz(&w, "scale");  cbor_w_double(&w, (double) s->map.analog.scale);
            cbor_w_textz(&w, "offset"); cbor_w_double(&w, (double) s->map.analog.offset);
        }
        if (is_generic) {
            uint8_t cc = s->map.sdi12.count <= SAVIA_SDI12_MAX_CHANNELS
                       ? s->map.sdi12.count : SAVIA_SDI12_MAX_CHANNELS;
            cbor_w_textz(&w, "chan");
            cbor_w_array(&w, cc);
            for (uint8_t c = 0; c < cc; c++) {
                cbor_w_map(&w, 2);
                cbor_w_textz(&w, "kind");     cbor_w_textz(&w, kind_str(s->map.sdi12.ch[c].kind));
                cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, s->map.sdi12.ch[c].depth_cm);
            }
        }
    }

    return w.overflow ? 0 : w.len;
}

// --- pinmap characteristic (0015) -------------------------------------------

static const char *pin_state_str(uint8_t s) {
    switch (s) {
        case SAVIA_PIN_IN_USE:   return "in_use";
        case SAVIA_PIN_RESERVED: return "reserved";
        default:                 return "free";
    }
}

static const char *pin_reason_str(uint8_t r) {
    switch (r) {
        case SAVIA_PIN_REASON_SENSOR:    return "sensor";
        case SAVIA_PIN_REASON_WIRELESS:  return "wireless";
        case SAVIA_PIN_REASON_WAKE_BTN:  return "wake_btn";
        case SAVIA_PIN_REASON_LORA_UART: return "lora_uart";
        default:                         return "";
    }
}

size_t ble_serialize_pinmap(const station_config_t *cfg, uint8_t *out, size_t cap) {
    savia_pin_info_t pins[SAVIA_GPIO_COUNT];
    pinmap_build(cfg, pins);

    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 2);
    cbor_w_textz(&w, "v");    cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
    cbor_w_textz(&w, "pins"); cbor_w_array(&w, SAVIA_GPIO_COUNT);
    for (uint8_t g = 0; g < SAVIA_GPIO_COUNT; g++) {
        bool is_sensor = pins[g].reason == SAVIA_PIN_REASON_SENSOR;
        cbor_w_map(&w, is_sensor ? 5 : 4);   // +port only when a sensor lives here
        cbor_w_textz(&w, "gpio");   cbor_w_uint(&w, pins[g].gpio);
        cbor_w_textz(&w, "state");  cbor_w_textz(&w, pin_state_str(pins[g].state));
        cbor_w_textz(&w, "reason"); cbor_w_textz(&w, pin_reason_str(pins[g].reason));
        cbor_w_textz(&w, "caps");   cbor_w_uint(&w, pins[g].caps);   // savia_pin_cap_t bits
        if (is_sensor) { cbor_w_textz(&w, "port"); cbor_w_uint(&w, pins[g].port); }
    }
    return w.overflow ? 0 : w.len;
}

// Parse one sensor entry {gpio, type, addr?, interval_s?, kind?, depth_cm?, scale?,
// offset?, chan?:[{kind,depth_cm}]} into *slot. "port" and unknown keys are skipped. An
// unknown type string yields SENSOR_NONE. Returns false only on malformed CBOR.
static bool parse_sensor_slot(cbor_reader_t *r, savia_sensor_slot_t *slot) {
    memset(slot, 0, sizeof(*slot));
    bool has_kind = false;
    // Buffer the type-specific (union) fields and resolve them against `type` only
    // AFTER the whole map is read: CBOR key order is not guaranteed, and writing both
    // union arms (analog scale/offset and the sdi12 chan[]) directly would alias --
    // a slot carrying both would corrupt map.sdi12.count. So commit one arm at the end.
    float a_scale = 0.0f, a_offset = 0.0f;
    savia_channel_t chans[SAVIA_SDI12_MAX_CHANNELS];
    uint8_t chan_count = 0;
    memset(chans, 0, sizeof(chans));

    uint64_t fcount;
    if (!cbor_r_map(r, &fcount)) return false;
    for (uint64_t f = 0; ; f++) {
        if (fcount == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(r)) break; }
        else if (f >= fcount) break;
        const char *fk; size_t fkn;
        if (!cbor_r_text(r, &fk, &fkn)) return false;
        if (cbor_text_eq(fk, fkn, "gpio")) {
            uint64_t v; if (!cbor_r_uint(r, &v)) return false; slot->gpio = (uint8_t) v;
        } else if (cbor_text_eq(fk, fkn, "type")) {
            const char *s; size_t sn; if (!cbor_r_text(r, &s, &sn)) return false;
            slot->type = sensor_type_from_str(s, sn);
        } else if (cbor_text_eq(fk, fkn, "addr")) {
            const char *s; size_t sn; if (!cbor_r_text(r, &s, &sn)) return false;
            slot->address = sn > 0 ? s[0] : 0;
        } else if (cbor_text_eq(fk, fkn, "kind")) {
            const char *s; size_t sn; if (!cbor_r_text(r, &s, &sn)) return false;
            int k = kind_from_str(s, sn);
            if (k >= 0) { slot->kind = (uint8_t) k; has_kind = true; }
        } else if (cbor_text_eq(fk, fkn, "depth_cm")) {
            if (!cbor_r_null(r)) { uint64_t v; if (!cbor_r_uint(r, &v)) return false; slot->depth_cm = (uint8_t) v; }
        } else if (cbor_text_eq(fk, fkn, "interval_s")) {
            if (!cbor_r_null(r)) { uint64_t v; if (!cbor_r_uint(r, &v)) return false; slot->sample_interval_s = (uint32_t) v; }
        } else if (cbor_text_eq(fk, fkn, "scale")) {
            double d; if (!cbor_r_double(r, &d)) return false; a_scale = (float) d;
        } else if (cbor_text_eq(fk, fkn, "offset")) {
            double d; if (!cbor_r_double(r, &d)) return false; a_offset = (float) d;
        } else if (cbor_text_eq(fk, fkn, "chan")) {
            uint64_t ccount;
            if (!cbor_r_array(r, &ccount)) return false;
            for (uint64_t c = 0; ; c++) {
                if (ccount == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(r)) break; }
                else if (c >= ccount) break;
                uint64_t mc;
                if (!cbor_r_map(r, &mc)) return false;
                savia_channel_t ch = { 0, 0 };
                for (uint64_t m = 0; ; m++) {
                    if (mc == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(r)) break; }
                    else if (m >= mc) break;
                    const char *ck; size_t ckn;
                    if (!cbor_r_text(r, &ck, &ckn)) return false;
                    if (cbor_text_eq(ck, ckn, "kind")) {
                        const char *s; size_t sn; if (!cbor_r_text(r, &s, &sn)) return false;
                        int k = kind_from_str(s, sn); if (k >= 0) ch.kind = (uint8_t) k;
                    } else if (cbor_text_eq(ck, ckn, "depth_cm")) {
                        uint64_t v; if (!cbor_r_uint(r, &v)) return false; ch.depth_cm = (uint8_t) v;
                    } else if (!cbor_r_skip(r)) return false;
                }
                if (chan_count < SAVIA_SDI12_MAX_CHANNELS) chans[chan_count++] = ch;  // extras dropped
            }
        } else if (!cbor_r_skip(r)) return false;   // "port" and unknowns
    }

    // Commit ONLY the union arm that matches the resolved type; the other arm stays
    // zeroed (from the memset), so a slot can never hold two arms at once.
    if (slot->type == SENSOR_ANALOG_LINEAR) {
        slot->map.analog.scale = a_scale;
        slot->map.analog.offset = a_offset;
    } else if (slot->type == SENSOR_SDI12_GENERIC) {
        slot->map.sdi12.count = chan_count;
        for (uint8_t i = 0; i < chan_count; i++) slot->map.sdi12.ch[i] = chans[i];
    }
    // Sensible default kind for a 1-Wire thermometer when the app omits it.
    if (slot->type == SENSOR_ONEWIRE_DS18B20 && !has_kind) slot->kind = READING_SOIL_TEMPERATURE;
    return true;
}

bool ble_parse_config_patch(const uint8_t *buf, size_t len, ble_config_patch_t *out) {
    memset(out, 0, sizeof(*out));
    cbor_reader_t r;
    cbor_r_init(&r, buf, len);
    uint64_t count;
    if (!cbor_r_map(&r, &count)) return false;
    for (uint64_t i = 0; ; i++) {
        if (count == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
        else if (i >= count) break;
        const char *k; size_t kn;
        if (!cbor_r_text(&r, &k, &kn)) return false;
        if (cbor_text_eq(k, kn, "v")) {
            uint64_t v; if (!cbor_r_uint(&r, &v)) return false; out->version = (int) v;
        } else if (cbor_text_eq(k, kn, "op")) {
            const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
            if (sn >= sizeof(out->op)) sn = sizeof(out->op) - 1;
            memcpy(out->op, s, sn); out->op[sn] = 0;
        } else if (cbor_text_eq(k, kn, "name")) {
            if (!cbor_r_null(&r)) {
                const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
                if (sn >= sizeof(out->name)) sn = sizeof(out->name) - 1;
                memcpy(out->name, s, sn); out->name[sn] = 0; out->has_name = true;
            }
        } else if (cbor_text_eq(k, kn, "sleep_s")) {
            if (!cbor_r_null(&r)) {
                uint64_t v; if (!cbor_r_uint(&r, &v)) return false;
                out->sleep_s = (uint32_t) v; out->has_sleep_s = true;
            }
        } else if (cbor_text_eq(k, kn, "deep_sleep")) {
            bool b;
            if (cbor_r_bool(&r, &b)) { out->deep_sleep = b; out->has_deep_sleep = true; }
            else if (!cbor_r_skip(&r)) return false;   // null/other -> ignore
        } else if (cbor_text_eq(k, kn, "capture_s")) {
            if (!cbor_r_null(&r)) {
                uint64_t v; if (!cbor_r_uint(&r, &v)) return false;
                out->capture_s = (uint32_t) v; out->has_capture_s = true;
            }
        } else if (cbor_text_eq(k, kn, "daily_hour")) {
            if (!cbor_r_null(&r)) {
                uint64_t v; if (!cbor_r_uint(&r, &v)) return false;
                out->daily_hour = (uint8_t) v; out->has_daily_hour = true;
            }
        } else if (cbor_text_eq(k, kn, "mock")) {
            bool b;
            if (cbor_r_bool(&r, &b)) { out->mock = b; out->has_mock = true; }
            else if (!cbor_r_skip(&r)) return false;
        } else if (cbor_text_eq(k, kn, "log_level")) {
            if (!cbor_r_null(&r)) {
                uint64_t v; if (!cbor_r_uint(&r, &v)) return false;
                out->log_level = (uint8_t) v; out->has_log_level = true;
            }
        } else if (cbor_text_eq(k, kn, "sensors")) {
            if (!cbor_r_null(&r)) {                       // null -> leave slots untouched
                uint64_t acount;
                if (!cbor_r_array(&r, &acount)) return false;
                uint8_t ni = 0;
                for (uint64_t j = 0; ; j++) {
                    if (acount == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
                    else if (j >= acount) break;
                    savia_sensor_slot_t slot;
                    if (!parse_sensor_slot(&r, &slot)) return false;
                    if (ni < SAVIA_MAX_SENSORS) out->sensors[ni++] = slot;   // extras dropped
                }
                out->sensor_count = ni;
                out->has_sensors = true;
            }
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    out->ok = !r.err;
    return out->ok;
}

size_t ble_serialize_logs(const char *const *lines, size_t n, uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_array(&w, n);
    for (size_t i = 0; i < n; i++) cbor_w_textz(&w, lines[i] ? lines[i] : "");
    return w.overflow ? 0 : w.len;
}

// --- auth characteristic (0014) ---------------------------------------------

size_t ble_serialize_auth_state(bool prov, bool authed,
                                const uint8_t *nonce, size_t nonce_len,
                                uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 4);
    cbor_w_textz(&w, "v");      cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
    cbor_w_textz(&w, "prov");   cbor_w_bool(&w, prov);
    cbor_w_textz(&w, "authed"); cbor_w_bool(&w, authed);
    cbor_w_textz(&w, "nonce");  cbor_w_bytes(&w, nonce, nonce_len);
    return w.overflow ? 0 : w.len;
}

// Copy a 32-byte field; returns true and sets *has on an exact-length match.
static void take_bytes32(cbor_reader_t *r, uint8_t out[32], bool *has) {
    const uint8_t *p; size_t n;
    if (cbor_r_bytes(r, &p, &n) && n == 32) { memcpy(out, p, 32); *has = true; }
}

bool ble_parse_auth(const uint8_t *buf, size_t len, ble_auth_msg_t *out) {
    memset(out, 0, sizeof(*out));
    cbor_reader_t r;
    cbor_r_init(&r, buf, len);
    uint64_t count;
    if (!cbor_r_map(&r, &count)) return false;
    for (uint64_t i = 0; ; i++) {
        if (count == SAVIA_CBOR_INDEFINITE) { if (cbor_r_at_break(&r)) break; }
        else if (i >= count) break;
        const char *k; size_t kn;
        if (!cbor_r_text(&r, &k, &kn)) return false;
        if (cbor_text_eq(k, kn, "v")) {
            uint64_t v; if (!cbor_r_uint(&r, &v)) return false; out->version = (int) v;
        } else if (cbor_text_eq(k, kn, "op")) {
            const char *s; size_t sn; if (!cbor_r_text(&r, &s, &sn)) return false;
            if (sn >= sizeof(out->op)) sn = sizeof(out->op) - 1;
            memcpy(out->op, s, sn); out->op[sn] = 0;
        } else if (cbor_text_eq(k, kn, "key")) {
            take_bytes32(&r, out->key, &out->has_key);
        } else if (cbor_text_eq(k, kn, "mac")) {
            take_bytes32(&r, out->mac, &out->has_mac);
        } else if (cbor_text_eq(k, kn, "old_mac")) {
            take_bytes32(&r, out->old_mac, &out->has_old_mac);
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    out->ok = !r.err;
    return out->ok;
}

size_t ble_serialize_config_ack(bool ok, uint32_t sleep_s, bool deep_sleep,
                                const char *err, uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    if (ok) {
        cbor_w_map(&w, 4);
        cbor_w_textz(&w, "v");          cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
        cbor_w_textz(&w, "op");         cbor_w_textz(&w, SAVIA_OP_CONFIG_OK);
        cbor_w_textz(&w, "sleep_s");    cbor_w_uint(&w, sleep_s);
        cbor_w_textz(&w, "deep_sleep"); cbor_w_bool(&w, deep_sleep);
    } else {
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "v");   cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
        cbor_w_textz(&w, "op");  cbor_w_textz(&w, SAVIA_OP_CONFIG_ERR);
        cbor_w_textz(&w, "msg"); cbor_w_textz(&w, err ? err : "invalid");
    }
    return w.overflow ? 0 : w.len;
}
