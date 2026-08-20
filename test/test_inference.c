// Host test for the LSTM input pipeline: StandardScaler + window gathering
// (soil aggregates + TA forecast) + tensor build + output un-scaling.
// Pure logic, no SDK / hardware.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "savia/scaler.h"
#include "savia/lstm_input.h"
#include "savia/storage.h"
#include "savia/weather.h"
#include "savia/types.h"

#define HOUR_MS 3600000ULL

// Scaler constants replicated from scaler_params.json for independent checks.
#define MU_HS30  0.7712527688624472
#define SD_HS30  0.042382551037717174
#define MU_TA    24.38746466771412
#define SD_TA    5.069760003904925
#define MU_HS10  0.7902161245631403
#define SD_HS10  0.04550010247318015

static int close_to(float a, double b) { return fabs((double) a - b) < 1e-4; }

// Seed `hours` hourly HS10/HS30 samples whose NEWEST lands in the hour `latest`
// (one reading per hour, mid-hour so it sits squarely inside its bucket). Pass a
// `latest` in the past to simulate a probe that stopped reporting back then.
static void seed_soil_hours(uint64_t latest, int hours) {
    for (int age = hours - 1; age >= 0; age--) {
        uint64_t ts = latest - (uint64_t) age * HOUR_MS + 60000ULL;
        savia_reading_t r10 = { .ts_ms = ts, .port = 1, .depth_cm = 10,
                                .kind = READING_SOIL_MOISTURE, .value = 0.70f };
        savia_reading_t r30 = { .ts_ms = ts, .port = 1, .depth_cm = 30,
                                .kind = READING_SOIL_MOISTURE, .value = 0.74f };
        storage_append_reading(&r10);
        storage_append_reading(&r30);
    }
}

int main(void) {
    // --- scaler forward + inverse ---
    assert(close_to(scaler_transform(0.80f, SCALER_HS30), (0.80 - MU_HS30) / SD_HS30));
    assert(close_to(scaler_transform(25.0f, SCALER_TA),   (25.0 - MU_TA) / SD_TA));
    assert(close_to(scaler_transform(0.78f, SCALER_HS10), (0.78 - MU_HS10) / SD_HS10));
    // roundtrip HS30
    float s = scaler_transform(0.66f, SCALER_HS30);
    assert(close_to(scaler_inverse_hs30(s), 0.66));
    printf("test_inference: scaler OK\n");

    // --- gather: 48 complete hours of HS10/HS30 + full weather cache ---
    storage_init();
    const uint64_t now = 1782000000000ULL;            // arbitrary "now"
    const uint64_t latest = now - (now % HOUR_MS);
    for (int age = 47; age >= 0; age--) {             // seed oldest -> newest
        uint64_t ts = latest - (uint64_t) age * HOUR_MS + 60000ULL;   // mid-hour
        savia_reading_t r10 = { .ts_ms = ts, .port = 1, .depth_cm = 10,
                                .kind = READING_SOIL_MOISTURE, .value = 0.70f + 0.001f * (47 - age) };
        savia_reading_t r30 = { .ts_ms = ts, .port = 1, .depth_cm = 30,
                                .kind = READING_SOIL_MOISTURE, .value = 0.74f + 0.001f * (47 - age) };
        storage_append_reading(&r10);
        storage_append_reading(&r30);
    }
    float pta[WEATHER_PAST_MAX], fta[WEATHER_FUTURE_MAX];
    for (int i = 0; i < WEATHER_PAST_MAX; i++)   pta[i] = 20.0f + 0.05f * i;
    for (int i = 0; i < WEATHER_FUTURE_MAX; i++) fta[i] = 22.0f + 0.10f * i;
    weather_set(pta, WEATHER_PAST_MAX, fta, WEATHER_FUTURE_MAX, now);

    lstm_raw_inputs_t raw;
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);
    // newest sample lands at index 47, oldest at 0
    assert(close_to(raw.hs30[47], 0.74f + 0.001f * 47));
    assert(close_to(raw.hs30[0],  0.74f));
    assert(close_to(raw.hs10[47], 0.70f + 0.001f * 47));
    assert(close_to(raw.ta[47],   pta[WEATHER_PAST_MAX - 1]));  // newest past TA
    assert(close_to(raw.future_ta[0], fta[0]));
    assert(close_to(raw.future_ta[23], fta[23]));
    printf("test_inference: gather OK\n");

    // --- LOCF: a missing interior hour is carried forward from the previous one ---
    storage_init();
    for (int age = 47; age >= 0; age--) {
        if (age == 20) continue;                      // drop hour at age 20
        uint64_t ts = latest - (uint64_t) age * HOUR_MS + 60000ULL;
        savia_reading_t r10 = { .ts_ms = ts, .port = 1, .depth_cm = 10,
                                .kind = READING_SOIL_MOISTURE, .value = 0.50f + 0.01f * (47 - age) };
        savia_reading_t r30 = { .ts_ms = ts, .port = 1, .depth_cm = 30,
                                .kind = READING_SOIL_MOISTURE, .value = 0.50f + 0.01f * (47 - age) };
        storage_append_reading(&r10);
        storage_append_reading(&r30);
    }
    weather_set(pta, WEATHER_PAST_MAX, fta, WEATHER_FUTURE_MAX, now);
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);
    size_t gap = LSTM_PAST_STEPS - 1 - 20;            // index of the dropped hour
    assert(close_to(raw.hs30[gap], raw.hs30[gap - 1]));   // carried forward
    printf("test_inference: LOCF OK\n");

    // --- missing forecast -> NO_FORECAST; missing soil -> INSUFFICIENT_HISTORY ---
    storage_init();
    seed_soil_hours(latest, LSTM_PAST_STEPS);         // soil fine, so the forecast decides
    weather_set(pta, 10, fta, 5, now);                // too few samples
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_NO_FORECAST);

    storage_init();
    weather_set(pta, WEATHER_PAST_MAX, fta, WEATHER_FUTURE_MAX, now);   // forecast fine
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_INSUFFICIENT_HISTORY);  // no soil
    printf("test_inference: missing-input guards OK\n");

    // --- real-coverage floor: the window must not be manufactured by LOCF ---
    // One hour of readings used to back-fill all 48 slots and report OK, so a
    // station an hour old "had" two days of history. Sub-floor coverage now fails.
    storage_init();
    weather_set(pta, WEATHER_PAST_MAX, fta, WEATHER_FUTURE_MAX, now);
    seed_soil_hours(latest, 1);                       // a single hour
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_INSUFFICIENT_HISTORY);

    storage_init();
    seed_soil_hours(latest, LSTM_MIN_PAST_HOURS - 1); // one short of the floor
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_INSUFFICIENT_HISTORY);

    storage_init();
    seed_soil_hours(latest, LSTM_MIN_PAST_HOURS);     // exactly the floor -> accepted
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);
    assert(close_to(raw.hs30[0], 0.74f));             // leading gap back-filled

    // Readings packed inside ONE hour are ONE bucket, however many there are: 60
    // samples a minute apart must not look like 60 hours of history.
    storage_init();
    for (int m = 0; m < 60; m++) {
        savia_reading_t r10 = { .ts_ms = latest + (uint64_t) m * 60000ULL, .port = 1,
                                .depth_cm = 10, .kind = READING_SOIL_MOISTURE, .value = 0.70f };
        savia_reading_t r30 = r10; r30.depth_cm = 30; r30.value = 0.74f;
        storage_append_reading(&r10);
        storage_append_reading(&r30);
    }
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_INSUFFICIENT_HISTORY);
    printf("test_inference: real-coverage floor OK\n");

    // --- staleness bound: plenty of history, but it stops hours ago ---
    // The probe reported for 43 h and then went quiet 5 h before the inference.
    // Coverage is 43/48, way over the floor -- only the staleness check sees it.
    storage_init();
    weather_set(pta, WEATHER_PAST_MAX, fta, WEATHER_FUTURE_MAX, now);
    seed_soil_hours(latest - 5 * HOUR_MS, 43);
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_STALE_HISTORY);

    // Exactly at the bound still passes (with tolerance 0: the current hour is
    // real). This is what "sample before you infer" guarantees -- see scheduler.c.
    storage_init();
    seed_soil_hours(latest - LSTM_MAX_STALE_HOURS * HOUR_MS, 30);
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);

    // One hour past the bound already fails: the newest step would be a copy.
    storage_init();
    seed_soil_hours(latest - (LSTM_MAX_STALE_HOURS + 1) * HOUR_MS, 30);
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_STALE_HISTORY);

    // Only one of the two depths going quiet is enough to refuse: the model reads
    // both, so a half-dead probe is not a usable window either.
    storage_init();
    seed_soil_hours(latest, 30);                              // HS10 + HS30 fresh
    for (int age = 29; age >= 0; age--) {                     // extra HS10 only
        savia_reading_t r10 = { .ts_ms = latest - (uint64_t) age * HOUR_MS + 60000ULL,
                                .port = 1, .depth_cm = 10,
                                .kind = READING_SOIL_MOISTURE, .value = 0.70f };
        storage_append_reading(&r10);
    }
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);   // both still fresh

    // A gap in the MIDDLE is what LOCF is for and must still pass, however wide,
    // as long as the newest bucket is fresh and coverage clears the floor.
    storage_init();
    seed_soil_hours(latest - 20 * HOUR_MS, 28);   // oldest block, ends 20 h back
    seed_soil_hours(latest, 20);                  // fresh block, up to now
    assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);
    printf("test_inference: staleness bound OK\n");

    // --- build tensors: scaling + model column order [TA, HS10, HS30] ---
    memset(&raw, 0, sizeof(raw));
    for (int t = 0; t < LSTM_PAST_STEPS; t++) {
        raw.ta[t] = 25.0f; raw.hs10[t] = 0.78f; raw.hs30[t] = 0.80f;
    }
    for (int t = 0; t < LSTM_FUTURE_STEPS; t++) raw.future_ta[t] = 23.0f;
    float past[LSTM_PAST_STEPS * LSTM_PAST_FEATURES];
    float future[LSTM_FUTURE_STEPS];
    lstm_build_tensors(&raw, past, future);
    // row 0: [TA, HS10, HS30]
    assert(close_to(past[LSTM_FEAT_TA],   (25.0 - MU_TA) / SD_TA));
    assert(close_to(past[LSTM_FEAT_HS10], (0.78 - MU_HS10) / SD_HS10));
    assert(close_to(past[LSTM_FEAT_HS30], (0.80 - MU_HS30) / SD_HS30));
    assert(close_to(future[0], (23.0 - MU_TA) / SD_TA));
    printf("test_inference: build_tensors OK\n");

    // --- un-scale output roundtrips through the HS30 scaler ---
    float scaled[LSTM_OUTPUT_STEPS], real[LSTM_OUTPUT_STEPS];
    for (int t = 0; t < LSTM_OUTPUT_STEPS; t++) scaled[t] = scaler_transform(0.72f, SCALER_HS30);
    lstm_unscale_output(scaled, real);
    for (int t = 0; t < LSTM_OUTPUT_STEPS; t++) assert(close_to(real[t], 0.72));
    printf("test_inference: unscale OK\n");

    printf("test_inference: OK\n");
    return 0;
}
