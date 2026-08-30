#include "savia/weather.h"
#include <string.h>
#include <stddef.h>

// In-RAM weather cache (the LSTM's TA window). Single-writer (LoRa cycle / BLE),
// read by the inference path -- same role as savia_py's `state` weather dict.

static float    s_past[WEATHER_PAST_MAX];
static float    s_future[WEATHER_FUTURE_MAX];
static uint8_t  s_n_past;
static uint8_t  s_n_future;
static uint64_t s_updated_ms;
static bool     s_set;

void weather_set(const float *past_ta, uint8_t n_past,
                 const float *future_ta, uint8_t n_future, uint64_t updated_ms) {
    if (n_past > WEATHER_PAST_MAX)     n_past = WEATHER_PAST_MAX;
    if (n_future > WEATHER_FUTURE_MAX) n_future = WEATHER_FUTURE_MAX;
    if (past_ta && n_past)     memcpy(s_past, past_ta, (size_t) n_past * sizeof(float));
    if (future_ta && n_future) memcpy(s_future, future_ta, (size_t) n_future * sizeof(float));
    s_n_past = n_past;
    s_n_future = n_future;
    s_updated_ms = updated_ms;
    s_set = true;
}

uint8_t weather_get_past(const float **out) {
    if (out) *out = s_past;
    return s_n_past;
}

uint8_t weather_get_future(const float **out) {
    if (out) *out = s_future;
    return s_n_future;
}

uint64_t weather_updated_ms(void) { return s_updated_ms; }

bool weather_is_set(void) { return s_set; }

// --- Persistence halves -----------------------------------------------------

static void wr_u64_le(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t) (v >> (8 * i));
}
static uint64_t rd_u64_le(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t) p[i] << (8 * i);
    return v;
}
static void wr_f32_le(uint8_t *p, float f) {
    uint32_t b; memcpy(&b, &f, sizeof b);
    for (int i = 0; i < 4; i++) p[i] = (uint8_t) (b >> (8 * i));
}
static float rd_f32_le(const uint8_t *p) {
    uint32_t b = 0;
    for (int i = 0; i < 4; i++) b |= (uint32_t) p[i] << (8 * i);
    float f; memcpy(&f, &b, sizeof f);
    return f;
}

size_t weather_serialize(uint8_t *out, size_t cap) {
    if (!s_set) return 0;
    size_t need = 2 + 8 + (size_t) (s_n_past + s_n_future) * 4;
    if (!out || cap < need) return 0;
    size_t p = 0;
    out[p++] = s_n_past;
    out[p++] = s_n_future;
    wr_u64_le(out + p, s_updated_ms); p += 8;
    for (uint8_t i = 0; i < s_n_past; i++)   { wr_f32_le(out + p, s_past[i]);   p += 4; }
    for (uint8_t i = 0; i < s_n_future; i++) { wr_f32_le(out + p, s_future[i]); p += 4; }
    return p;
}

bool weather_restore(const uint8_t *data, size_t len, uint64_t now_ms, uint64_t max_age_ms) {
    if (!data || len < 10) return false;
    uint8_t n_past = data[0], n_future = data[1];
    if (n_past > WEATHER_PAST_MAX || n_future > WEATHER_FUTURE_MAX) return false;
    if (len < (size_t) 10 + (size_t) (n_past + n_future) * 4) return false;
    uint64_t updated = rd_u64_le(data + 2);
    // Unknown or future-dated stamp, or simply too old for the hour we are in:
    // an unaligned TA window is worse than none, because nothing downstream
    // would catch it.
    if (updated == 0 || now_ms < updated) return false;
    if (now_ms - updated > max_age_ms) return false;
    size_t p = 10;
    for (uint8_t i = 0; i < n_past; i++)   { s_past[i]   = rd_f32_le(data + p); p += 4; }
    for (uint8_t i = 0; i < n_future; i++) { s_future[i] = rd_f32_le(data + p); p += 4; }
    s_n_past = n_past;
    s_n_future = n_future;
    s_updated_ms = updated;
    s_set = true;
    return true;
}
