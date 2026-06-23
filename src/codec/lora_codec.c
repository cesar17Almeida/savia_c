#include "savia/lora_codec.h"

// Byte codec for the LoRaWAN node payloads. Kept in lockstep with savia_py's
// lora_codec.py (the backend speaks the same format). No SDK -> host-testable.

// --- int8 TA helpers (round + clamp, no <math.h>) ---------------------------

static int8_t to_int8(float celsius) {
    float r = celsius >= 0.0f ? celsius + 0.5f : celsius - 0.5f;
    int v = (int) r;
    if (v < -128) v = -128;
    if (v > 127)  v = 127;
    return (int8_t) v;
}

static void put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t) (v >> 24); p[1] = (uint8_t) (v >> 16);
    p[2] = (uint8_t) (v >> 8);  p[3] = (uint8_t) v;
}

static uint32_t get_be32(const uint8_t *p) {
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) |
           ((uint32_t) p[2] << 8)  |  (uint32_t) p[3];
}

// --- downlink (backend -> node) ---------------------------------------------

size_t lora_encode_downlink(const float *past_ta, uint8_t n_past,
                            const float *future_ta, uint8_t n_future,
                            bool has_time, uint64_t time_ms,
                            uint8_t *out, size_t cap) {
    if (n_past > LORA_TA_PAST_MAX || n_future > LORA_TA_FUTURE_MAX) return 0;
    size_t need = 7u + n_past + n_future;
    if (cap < need) return 0;

    uint32_t time_s = 0;
    if (has_time) {
        uint64_t s = time_ms / 1000u;
        time_s = s > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t) s;
    }
    size_t i = 0;
    out[i++] = LORA_CODEC_VERSION;
    put_be32(&out[i], time_s); i += 4;
    out[i++] = n_past;
    out[i++] = n_future;
    for (uint8_t k = 0; k < n_past; k++)   out[i++] = (uint8_t) to_int8(past_ta[k]);
    for (uint8_t k = 0; k < n_future; k++) out[i++] = (uint8_t) to_int8(future_ta[k]);
    return i;
}

bool lora_decode_downlink(const uint8_t *data, size_t len, lora_downlink_t *out) {
    if (len < 7 || data[0] != LORA_CODEC_VERSION) return false;
    uint8_t n_past = data[5], n_future = data[6];
    if (n_past > LORA_TA_PAST_MAX || n_future > LORA_TA_FUTURE_MAX) return false;
    size_t need = 7u + n_past + n_future;
    if (len < need) return false;

    uint32_t time_s = get_be32(&data[1]);
    out->has_time = time_s > 0;
    out->time_ms = (uint64_t) time_s * 1000u;
    out->n_past = n_past;
    out->n_future = n_future;
    size_t off = 7;
    for (uint8_t k = 0; k < n_past; k++)   out->past_ta[k]   = (float) (int8_t) data[off++];
    for (uint8_t k = 0; k < n_future; k++) out->future_ta[k] = (float) (int8_t) data[off++];
    return true;
}

// --- uplink (node -> backend) -----------------------------------------------

#define HS30_UNKNOWN 0xFFFFu

size_t lora_encode_uplink(bool has_hs30, float hs30_min, uint8_t *out, size_t cap) {
    if (cap < LORA_UPLINK_LEN) return 0;
    uint16_t hs = HS30_UNKNOWN;
    if (has_hs30) {
        long v = (long) (hs30_min * 1000.0f + 0.5f);
        if (v < 0) v = 0;
        if (v > 0xFFFE) v = 0xFFFE;
        hs = (uint16_t) v;
    }
    out[0] = LORA_CODEC_VERSION;
    out[1] = (uint8_t) (hs >> 8);
    out[2] = (uint8_t) hs;
    return LORA_UPLINK_LEN;
}

bool lora_decode_uplink(const uint8_t *data, size_t len,
                        bool *has_hs30, float *hs30_min) {
    if (len < LORA_UPLINK_LEN || data[0] != LORA_CODEC_VERSION) return false;
    uint16_t raw = (uint16_t) (((uint16_t) data[1] << 8) | data[2]);
    *has_hs30 = raw != HS30_UNKNOWN;
    *hs30_min = *has_hs30 ? raw / 1000.0f : 0.0f;
    return true;
}
