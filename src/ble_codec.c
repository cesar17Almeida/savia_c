#include "savia/ble_codec.h"
#include "savia/cbor.h"
#include "savia/protocol.h"
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
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    out->ok = !r.err;
    return out->ok;
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
                            uint64_t weather_updated_ms, uint8_t *out, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, out, cap);
    cbor_w_map(&w, 5);
    cbor_w_textz(&w, "v");        cbor_w_uint(&w, SAVIA_PROTOCOL_VERSION);
    cbor_w_textz(&w, "fw");       cbor_w_textz(&w, SAVIA_FW_VERSION);
    cbor_w_textz(&w, "uptime_s"); cbor_w_uint(&w, uptime_s);
    cbor_w_textz(&w, "last_sync_ms");
    if (last_sync_ms) cbor_w_uint(&w, last_sync_ms); else cbor_w_null(&w);
    cbor_w_textz(&w, "weather_updated_ms");
    if (weather_updated_ms) cbor_w_uint(&w, weather_updated_ms); else cbor_w_null(&w);
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
        case SENSOR_SDI12_AQUACHECK: return "sdi12_aquacheck";
        case SENSOR_SDI12_GENERIC:   return "sdi12_generic";
        default:                     return "none";
    }
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

    // sensors: one entry per configured slot.
    uint8_t nsens = cfg->sensor_count <= SAVIA_MAX_SENSORS ? cfg->sensor_count : SAVIA_MAX_SENSORS;
    cbor_w_textz(&w, "sensors");
    cbor_w_array(&w, nsens);
    for (uint8_t i = 0; i < nsens; i++) {
        cbor_w_map(&w, 4);
        cbor_w_textz(&w, "port"); cbor_w_uint(&w, (uint64_t)(i + 1));
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, cfg->sensors[i].gpio);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, sensor_type_str(cfg->sensors[i].type));
        char addr[2] = { cfg->sensors[i].address, 0 };
        cbor_w_textz(&w, "addr"); cbor_w_textz(&w, addr);
    }

    return w.overflow ? 0 : w.len;
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
