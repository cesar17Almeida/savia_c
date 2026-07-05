#include "savia/lora_codec.h"
#include "savia/log.h"

// Byte codec for the LoRaWAN node payloads, wire v2. Python mirror in
// savia-cloud/app/adapters/ttn/codec.py -- keep the golden test vectors in sync.
// No SDK -> host-testable.

#define HS_UNKNOWN 0xFFFFu   // u16 x1000 sentinel (soil / forecast)
#define TA_UNKNOWN 0x7FFF    // i16 x10 sentinel

// --- big-endian + rounding helpers (no <math.h>) -----------------------------

static int8_t to_int8(float celsius) {
    float r = celsius >= 0.0f ? celsius + 0.5f : celsius - 0.5f;
    int v = (int) r;
    if (v < -128) v = -128;
    if (v > 127)  v = 127;
    return (int8_t) v;
}

static uint16_t vwc_to_u16(float vwc) {          // VWC 0..1 -> x1000, clamped
    long v = (long) (vwc * 1000.0f + 0.5f);
    if (v < 0) v = 0;
    if (v > 0xFFFE) v = 0xFFFE;
    return (uint16_t) v;
}

static int16_t ta_to_i16(float celsius) {        // degC -> x10, clamped
    float r = celsius >= 0.0f ? celsius * 10.0f + 0.5f : celsius * 10.0f - 0.5f;
    long v = (long) r;
    if (v < -32768) v = -32768;
    if (v > 0x7FFE) v = 0x7FFE;                  // 0x7FFF reserved as sentinel
    return (int16_t) v;
}

static void put_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t) v; }
static void put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t) v;
}
static uint16_t get_be16(const uint8_t *p) { return (uint16_t)(((uint16_t) p[0] << 8) | p[1]); }
static uint32_t get_be32(const uint8_t *p) {
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) |
           ((uint32_t) p[2] << 8)  |  (uint32_t) p[3];
}

// --- uplink encoders ----------------------------------------------------------

size_t lora_encode_uplink_forecast(bool has_hs30, float hs30_min,
                                   uint8_t *out, size_t cap) {
    if (cap < 4) return 0;
    out[0] = LORA_CODEC_VERSION;
    out[1] = LORA_UP_FORECAST;
    put_be16(&out[2], has_hs30 ? vwc_to_u16(hs30_min) : HS_UNKNOWN);
    return 4;
}

size_t lora_encode_uplink_soil(const lora_soil_rec_t *recs, uint8_t n,
                               uint8_t *out, size_t cap) {
    if (n == 0 || n > LORA_SOIL_RECS_MAX) return 0;
    size_t need = 3u + (size_t) n * 10u;
    if (cap < need) return 0;
    out[0] = LORA_CODEC_VERSION;
    out[1] = LORA_UP_SOIL;
    out[2] = n;
    size_t i = 3;
    for (uint8_t k = 0; k < n; k++) {
        const lora_soil_rec_t *r = &recs[k];
        put_be32(&out[i], r->ts_hour_s); i += 4;
        put_be16(&out[i], r->has_hs10 ? vwc_to_u16(r->hs10) : HS_UNKNOWN); i += 2;
        put_be16(&out[i], r->has_hs30 ? vwc_to_u16(r->hs30) : HS_UNKNOWN); i += 2;
        put_be16(&out[i], (uint16_t)(r->has_ta ? ta_to_i16(r->ta) : TA_UNKNOWN)); i += 2;
    }
    return i;
}

size_t lora_encode_uplink_coords(int32_t lat_e7, int32_t lon_e7,
                                 int16_t utc_offset_min, uint8_t *out, size_t cap) {
    if (cap < 12) return 0;
    out[0] = LORA_CODEC_VERSION;
    out[1] = LORA_UP_COORDS;
    put_be32(&out[2], (uint32_t) lat_e7);
    put_be32(&out[6], (uint32_t) lon_e7);
    put_be16(&out[10], (uint16_t) utc_offset_min);
    return 12;
}

size_t lora_encode_uplink_cfg_ack(uint8_t applied, uint8_t rejected,
                                  uint8_t *out, size_t cap) {
    if (cap < 4) return 0;
    out[0] = LORA_CODEC_VERSION;
    out[1] = LORA_UP_CFG_ACK;
    out[2] = applied;
    out[3] = rejected;
    return 4;
}

// --- downlink -----------------------------------------------------------------

size_t lora_encode_downlink_time_ta(const float *past_ta, uint8_t n_past,
                                    const float *future_ta, uint8_t n_future,
                                    bool has_time, uint64_t time_ms,
                                    uint8_t *out, size_t cap) {
    if (n_past > LORA_TA_PAST_MAX || n_future > LORA_TA_FUTURE_MAX) return 0;
    size_t need = 8u + n_past + n_future;
    if (cap < need) return 0;

    uint32_t time_s = 0;
    if (has_time) {
        uint64_t s = time_ms / 1000u;
        time_s = s > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t) s;
    }
    size_t i = 0;
    out[i++] = LORA_CODEC_VERSION;
    out[i++] = LORA_DN_TIME_TA;
    put_be32(&out[i], time_s); i += 4;
    out[i++] = n_past;
    out[i++] = n_future;
    for (uint8_t k = 0; k < n_past; k++)   out[i++] = (uint8_t) to_int8(past_ta[k]);
    for (uint8_t k = 0; k < n_future; k++) out[i++] = (uint8_t) to_int8(future_ta[k]);
    return i;
}

bool lora_decode_downlink(const uint8_t *data, size_t len, lora_downlink_t *out) {
    if (len < 2 || data[0] != LORA_CODEC_VERSION) return false;
    out->type = data[1];
    out->tlv = NULL;
    out->tlv_len = 0;

    if (out->type == LORA_DN_TIME_TA) {
        if (len < 8) return false;
        uint8_t n_past = data[6], n_future = data[7];
        if (n_past > LORA_TA_PAST_MAX || n_future > LORA_TA_FUTURE_MAX) return false;
        if (len < 8u + n_past + n_future) return false;
        uint32_t time_s = get_be32(&data[2]);
        out->has_time = time_s > 0;
        out->time_ms = (uint64_t) time_s * 1000u;
        out->n_past = n_past;
        out->n_future = n_future;
        size_t off = 8;
        for (uint8_t k = 0; k < n_past; k++)   out->past_ta[k]   = (float)(int8_t) data[off++];
        for (uint8_t k = 0; k < n_future; k++) out->future_ta[k] = (float)(int8_t) data[off++];
        return true;
    }
    if (out->type == LORA_DN_CONFIG) {
        out->has_time = false;
        out->n_past = out->n_future = 0;
        out->tlv = &data[2];
        out->tlv_len = len - 2;
        return true;
    }
    return false;   // unknown type
}

// --- config-patch TLV ----------------------------------------------------------

// Read one integer value of `len` bytes big-endian (1/2/4 accepted).
static bool tlv_value(const uint8_t *p, uint8_t len, uint32_t *v) {
    switch (len) {
        case 1: *v = p[0]; return true;
        case 2: *v = get_be16(p); return true;
        case 4: *v = get_be32(p); return true;
        default: return false;
    }
}

bool lora_apply_config_tlv(const uint8_t *tlv, size_t len, station_config_t *cfg,
                           uint8_t *applied, uint8_t *rejected) {
    uint8_t ok = 0, bad = 0;
    size_t i = 0;
    bool wellformed = true;

    while (i + 2 <= len) {
        uint8_t id = tlv[i], vlen = tlv[i + 1];
        if (i + 2u + vlen > len) { wellformed = false; break; }   // truncated value
        const uint8_t *vp = &tlv[i + 2];
        i += 2u + vlen;

        uint32_t v = 0;
        if (!tlv_value(vp, vlen, &v)) { bad++; continue; }        // bad length for id

        switch (id) {
            case LORA_CFG_SLEEP_S:
                if (v >= SAVIA_SLEEP_MIN_S && v <= SAVIA_SLEEP_MAX_S) { cfg->sleep_seconds = v; ok++; }
                else bad++;
                break;
            case LORA_CFG_DEEP_SLEEP:
                if (v <= 1) { cfg->deep_sleep_enabled = v != 0; ok++; } else bad++;
                break;
            case LORA_CFG_CAPTURE_S:
                if (v >= SAVIA_CAPTURE_MIN_S && v <= SAVIA_SLEEP_MAX_S) { cfg->capture_interval_s = v; ok++; }
                else bad++;
                break;
            case LORA_CFG_DAILY_HOUR:
                if (v <= 23) { cfg->daily_hour = (uint8_t) v; ok++; } else bad++;
                break;
            case LORA_CFG_LORA_PERIOD_S:
                if (v >= SAVIA_LORA_PERIOD_MIN_S && v <= SAVIA_LORA_PERIOD_MAX_S) { cfg->lora_period_s = v; ok++; }
                else bad++;
                break;
            case LORA_CFG_INFERENCE_MODE:
                // LOCAL is only meaningful on on-device builds; the CALLER enforces
                // that (needs inference_on_device(), not linkable here). Range only.
                if (v <= SAVIA_INFER_LOCAL) { cfg->inference_mode = (uint8_t) v; ok++; }
                else bad++;
                break;
            case LORA_CFG_UTC_OFFSET_MIN: {
                int16_t off = (int16_t)(uint16_t) v;
                if (vlen == 2 && off >= SAVIA_UTC_OFFSET_MIN && off <= SAVIA_UTC_OFFSET_MAX) {
                    cfg->utc_offset_min = off; ok++;
                } else bad++;
                break;
            }
            case LORA_CFG_IRRIGATION_HOUR:
                if (v <= 23) { cfg->irrigation_hour = (uint8_t) v; ok++; } else bad++;
                break;
            case LORA_CFG_LAT: {
                int32_t lat = (int32_t) v;
                if (vlen == 4 && lat >= -900000000 && lat <= 900000000) {
                    cfg->lat_e7 = lat; cfg->has_coords = true; ok++;
                } else bad++;
                break;
            }
            case LORA_CFG_LON: {
                int32_t lon = (int32_t) v;
                if (vlen == 4 && lon >= -1800000000 && lon <= 1800000000) {
                    cfg->lon_e7 = lon; cfg->has_coords = true; ok++;
                } else bad++;
                break;
            }
            case LORA_CFG_LOG_LEVEL:
                if (v <= 2) { cfg->log_level = (uint8_t) v; ok++; } else bad++;
                break;
            default:
                break;   // unknown id: skipped silently (forward-compat)
        }
    }
    if (i != len && wellformed) wellformed = false;   // trailing garbage (<2 bytes)

    if (applied)  *applied = ok;
    if (rejected) *rejected = bad;
    return wellformed;
}

// --- uplink decoders (tests / host tooling) ------------------------------------

bool lora_decode_uplink_forecast(const uint8_t *data, size_t len,
                                 bool *has_hs30, float *hs30_min) {
    if (len < 4 || data[0] != LORA_CODEC_VERSION || data[1] != LORA_UP_FORECAST)
        return false;
    uint16_t raw = get_be16(&data[2]);
    *has_hs30 = raw != HS_UNKNOWN;
    *hs30_min = *has_hs30 ? raw / 1000.0f : 0.0f;
    return true;
}
