#include "savia/lstm_input.h"
#include "savia/scaler.h"
#include "savia/storage.h"
#include "savia/weather.h"
#include "savia/types.h"
#include <stdbool.h>
#include <string.h>

#define HOUR_MS 3600000ULL

// Fill `series[48]` (oldest -> newest, index 47 == latest_hour) from the hourly
// aggregates matching (kind, depth). Missing interior/trailing buckets are carried
// forward (LOCF); a leading gap is back-filled from the first known sample.
// Returns false if NOTHING matched at all.
static bool fill_series(const savia_aggregate_t *aggs, size_t n,
                        uint64_t latest_hour, uint8_t kind, uint8_t depth_cm,
                        float *series) {
    bool present[LSTM_PAST_STEPS] = { false };
    for (size_t i = 0; i < n; i++) {
        if (aggs[i].kind != kind || aggs[i].depth_cm != depth_cm) continue;
        if (aggs[i].hour_ms > latest_hour) continue;
        uint64_t age = (latest_hour - aggs[i].hour_ms) / HOUR_MS;   // 0 = newest
        if (age >= LSTM_PAST_STEPS) continue;
        size_t idx = LSTM_PAST_STEPS - 1 - (size_t) age;
        series[idx] = aggs[i].mean;
        present[idx] = true;
    }
    int first = -1;
    for (size_t i = 0; i < LSTM_PAST_STEPS; i++) if (present[i]) { first = (int) i; break; }
    if (first < 0) return false;                                    // no data at all
    for (int i = 0; i < first; i++) series[i] = series[first];      // leading gap
    for (size_t i = (size_t) first + 1; i < LSTM_PAST_STEPS; i++)   // interior/trailing
        if (!present[i]) series[i] = series[i - 1];
    return true;
}

lstm_input_status_t lstm_gather_inputs(uint64_t now_ms, lstm_raw_inputs_t *out) {
    uint64_t latest_hour = now_ms - (now_ms % HOUR_MS);
    uint64_t from = latest_hour - (uint64_t)(LSTM_PAST_STEPS - 1) * HOUR_MS;
    uint64_t to   = latest_hour + HOUR_MS;   // half-open; includes the latest bucket

    // 48 h x up to a few series/depths; sized with margin.
    static savia_aggregate_t aggs[208];
    size_t n = storage_aggregate_hourly(from, to, 0, aggs, sizeof(aggs) / sizeof(aggs[0]));

    // HS10 / HS30 from the soil probe.
    if (!fill_series(aggs, n, latest_hour, READING_SOIL_MOISTURE, 10, out->hs10))
        return LSTM_INPUT_INSUFFICIENT_HISTORY;
    if (!fill_series(aggs, n, latest_hour, READING_SOIL_MOISTURE, 30, out->hs30))
        return LSTM_INPUT_INSUFFICIENT_HISTORY;

    // TA (past + future) from the weather cache. WEATHER_PAST_MAX / FUTURE_MAX are
    // exactly the LSTM window, so we require a full cache and align the newest past
    // sample to latest_hour, the first future sample to latest_hour + 1 h.
    const float *past_ta = NULL, *fut_ta = NULL;
    uint8_t n_past = weather_get_past(&past_ta);
    uint8_t n_fut  = weather_get_future(&fut_ta);
    if (!weather_is_set() || n_past < LSTM_PAST_STEPS || n_fut < LSTM_FUTURE_STEPS)
        return LSTM_INPUT_NO_FORECAST;
    memcpy(out->ta, past_ta + (n_past - LSTM_PAST_STEPS), LSTM_PAST_STEPS * sizeof(float));
    memcpy(out->future_ta, fut_ta, LSTM_FUTURE_STEPS * sizeof(float));
    return LSTM_INPUT_OK;
}

void lstm_build_tensors(const lstm_raw_inputs_t *in, float *past_out, float *future_out) {
    for (size_t t = 0; t < LSTM_PAST_STEPS; t++) {
        float *row = past_out + t * LSTM_PAST_FEATURES;            // model order [TA, HS10, HS30]
        row[LSTM_FEAT_TA]   = scaler_transform(in->ta[t],   SCALER_TA);
        row[LSTM_FEAT_HS10] = scaler_transform(in->hs10[t], SCALER_HS10);
        row[LSTM_FEAT_HS30] = scaler_transform(in->hs30[t], SCALER_HS30);
    }
    for (size_t t = 0; t < LSTM_FUTURE_STEPS; t++)
        future_out[t] = scaler_transform(in->future_ta[t], SCALER_TA);
}

void lstm_unscale_output(const float *scaled, float *hs30_real) {
    for (size_t t = 0; t < LSTM_OUTPUT_STEPS; t++)
        hs30_real[t] = scaler_inverse_hs30(scaled[t]);
}
