#include "savia/weather.h"
#include <string.h>

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
