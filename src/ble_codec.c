#include "savia/ble_codec.h"
#include "savia/cbor.h"
#include "savia/protocol.h"
#include <string.h>

static const char *kind_str(uint8_t kind) {
    switch (kind) {
        case READING_SOIL_MOISTURE:    return "soil_moisture";
        case READING_SOIL_TEMPERATURE: return "soil_temperature";
        default:                       return "unknown";
    }
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
    for (uint64_t i = 0; i < count; i++) {
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
            if (!cbor_r_uint(&r, &out->from_ms)) return false; out->has_from = true;
        } else if (cbor_text_eq(k, kn, "to")) {
            if (!cbor_r_uint(&r, &out->to_ms)) return false; out->has_to = true;
        } else if (cbor_text_eq(k, kn, "limit")) {
            if (!cbor_r_uint(&r, &out->limit)) return false; out->has_limit = true;
        } else {
            if (!cbor_r_skip(&r)) return false;
        }
    }
    out->ok = !r.err;
    return out->ok;
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
    for (uint64_t i = 0; i < count; i++) {
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
    for (uint64_t i = 0; i < count; i++) {
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
