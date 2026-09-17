// Host test for the mock soil probe: replay mapping, window fill and the hand-off
// to the LSTM input pipeline. Pure logic, no SDK / hardware.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "savia/mock_soil.h"
#include "savia/mock_soil_data.h"
#include "savia/lstm_input.h"
#include "savia/storage.h"
#include "savia/weather.h"
#include "savia/types.h"

#define HOUR_MS  3600000ULL
#define WINDOW_H (MOCK_SOIL_PAST_H + MOCK_SOIL_AHEAD_H)

static const uint64_t MIDNIGHT = 1789603200000ULL;   // 2026-09-17 00:00 UTC
static const uint64_t NOW      = 1789648496000ULL;   // 2026-09-17 12:34:56 UTC
static const int16_t  CEST     = 120;                // Spain in September

static float value(uint64_t ts, int16_t off, uint8_t depth) {
    float v = -1.0f;
    assert(mock_soil_value(ts, off, depth, &v));
    return v;
}

static void set_weather(uint64_t now) {
    float past[WEATHER_PAST_MAX], fut[WEATHER_FUTURE_MAX];
    for (int i = 0; i < WEATHER_PAST_MAX; i++)   past[i] = 20.0f + 0.1f * (float) i;
    for (int i = 0; i < WEATHER_FUTURE_MAX; i++) fut[i]  = 25.0f - 0.1f * (float) i;
    weather_set(past, WEATHER_PAST_MAX, fut, WEATHER_FUTURE_MAX, now);
}

static void test_mapping(void) {
    // A local midnight lands on a row that starts a day, and the replay then walks
    // one row per hour, wrapping after MOCK_SOIL_ROWS.
    uint32_t r0 = (uint32_t) ((MIDNIGHT / HOUR_MS) % MOCK_SOIL_ROWS);
    assert(r0 % 24 == 0);
    for (uint32_t k = 0; k < 2 * MOCK_SOIL_ROWS; k++) {
        uint64_t ts = MIDNIGHT + (uint64_t) k * HOUR_MS;
        assert(value(ts, 0, 10) == MOCK_SOIL_HS10[(r0 + k) % MOCK_SOIL_ROWS]);
        assert(value(ts, 0, 30) == MOCK_SOIL_HS30[(r0 + k) % MOCK_SOIL_ROWS]);
    }
    // Any minute of an hour reads that hour's row.
    assert(value(MIDNIGHT + 59 * 60000ULL, 0, 10) == value(MIDNIGHT, 0, 10));
    // Local time decides: 22:00 UTC is midnight at UTC+2, 05:00 UTC at UTC-5.
    assert(value(MIDNIGHT - 2 * HOUR_MS, CEST, 30) == value(MIDNIGHT, 0, 30));
    assert(value(MIDNIGHT + 5 * HOUR_MS, -300, 30) == value(MIDNIGHT, 0, 30));
    // Only the depths the model reads.
    float v;
    assert(!mock_soil_value(NOW, 0, 20, &v));
    assert(!mock_soil_value(NOW, 0, 40, &v));
    printf("test_mock_soil: replay mapping OK\n");
}

static void test_fill(void) {
    uint64_t cur = NOW - NOW % HOUR_MS;
    uint64_t first = cur - (MOCK_SOIL_PAST_H - 1) * HOUR_MS;

    // No wall clock yet: nothing to align the replay to.
    storage_init();
    assert(mock_soil_fill(123456, CEST) == 0);
    assert(storage_reading_count() == 0);

    // Empty ring: 48 h back + 24 h ahead, both depths, oldest first, on the hour.
    assert(mock_soil_fill(NOW, CEST) == 2 * WINDOW_H);
    assert(storage_reading_count() == 2 * WINDOW_H);
    for (size_t i = 0; i < storage_reading_count(); i++) {
        const savia_reading_t *r = storage_reading_at(i);
        assert(r->ts_ms == first + (i / 2) * HOUR_MS);
        assert(r->port == MOCK_SOIL_PORT && r->kind == READING_SOIL_MOISTURE);
        assert(r->depth_cm == (i % 2 ? 30 : 10));
        assert(r->value == value(r->ts_ms, CEST, r->depth_cm));
    }
    assert(storage_count_raw(cur + HOUR_MS, UINT64_MAX) == 2 * MOCK_SOIL_AHEAD_H);

    // Idempotent; as time moves on only the new hours are appended, still in order.
    assert(mock_soil_fill(NOW, CEST) == 0);
    assert(mock_soil_fill(NOW + HOUR_MS, CEST) == 2);
    assert(mock_soil_fill(NOW + 5 * HOUR_MS + 1200000ULL, CEST) == 8);   // 17:54
    assert(storage_reading_count() == 2 * WINDOW_H + 10);
    for (size_t i = 1; i < storage_reading_count(); i++)
        assert(storage_reading_at(i)->ts_ms >= storage_reading_at(i - 1)->ts_ms);

    // A reading already in an hour wins (an app ingest, say); other ports and
    // kinds do not count as the mock series.
    storage_init();
    savia_reading_t own = { .ts_ms = cur + 1800000ULL, .port = 1, .depth_cm = 10,
                            .kind = READING_SOIL_MOISTURE, .value = 0.5f };
    savia_reading_t other_port = { .ts_ms = cur, .port = 2, .depth_cm = 30,
                                   .kind = READING_SOIL_MOISTURE, .value = 0.1f };
    savia_reading_t other_kind = { .ts_ms = cur, .port = 1, .depth_cm = 30,
                                   .kind = READING_SOIL_TEMPERATURE, .value = 21.0f };
    storage_append_reading(&own);
    storage_append_reading(&other_port);
    storage_append_reading(&other_kind);
    assert(mock_soil_fill(NOW, CEST) == 2 * WINDOW_H - 1);
    savia_aggregate_t agg[4];
    assert(storage_aggregate_series(cur, cur + HOUR_MS, READING_SOIL_MOISTURE, 10, agg, 4) == 1);
    assert(agg[0].count == 1 && agg[0].mean == 0.5f);
    assert(storage_aggregate_series(cur, cur + HOUR_MS, READING_SOIL_MOISTURE, 30, agg, 4) == 1);
    assert(agg[0].port == 1 && agg[0].mean == value(cur, CEST, 30));
    printf("test_mock_soil: window fill OK\n");
}

static void test_lstm_handoff(void) {
    uint64_t cur = NOW - NOW % HOUR_MS;
    lstm_raw_inputs_t raw;

    // Soil only: without a TA forecast the model still cannot run.
    storage_init();
    assert(mock_soil_fill(NOW, CEST) > 0);
    assert(lstm_gather_inputs(NOW, &raw) == LSTM_INPUT_NO_FORECAST);

    // With the forecast the window is complete, and the future hours stay out of it.
    set_weather(NOW);
    assert(lstm_gather_inputs(NOW, &raw) == LSTM_INPUT_OK);
    for (int t = 0; t < LSTM_PAST_STEPS; t++) {
        uint64_t ts = cur - (uint64_t) (LSTM_PAST_STEPS - 1 - t) * HOUR_MS;
        assert(raw.hs10[t] == value(ts, CEST, 10));
        assert(raw.hs30[t] == value(ts, CEST, 30));
    }

    // Hour after hour, through a full ring and a replay wrap, it stays usable.
    storage_init();
    for (uint32_t h = 0; h < MOCK_SOIL_ROWS + 30; h++) {
        uint64_t now = NOW + (uint64_t) h * HOUR_MS;
        mock_soil_fill(now, CEST);
        if (h % 50 == 0 || h + 1 == MOCK_SOIL_ROWS + 30) {
            set_weather(now);
            assert(lstm_gather_inputs(now, &raw) == LSTM_INPUT_OK);
        }
    }
    assert(storage_reading_count() == SAVIA_READINGS_CAP);
    printf("test_mock_soil: LSTM hand-off OK\n");
}

static void test_selftest_window(void) {
    lstm_raw_inputs_t raw;
    const float *host = NULL, *truth = NULL;
    mock_soil_selftest_window(&raw, &host, &truth);
    for (int t = 0; t < LSTM_PAST_STEPS; t++) {
        assert(raw.hs10[t] == MOCK_SOIL_HS10[MOCK_SELFTEST_ROW + t]);
        assert(raw.hs30[t] == MOCK_SOIL_HS30[MOCK_SELFTEST_ROW + t]);
        assert(raw.ta[t] == MOCK_SELFTEST_TA[t]);
    }
    double mae = 0.0;
    for (int t = 0; t < LSTM_FUTURE_STEPS; t++) {
        assert(raw.future_ta[t] == MOCK_SELFTEST_TA[LSTM_PAST_STEPS + t]);
        assert(truth[t] == MOCK_SOIL_HS30[MOCK_SELFTEST_ROW + LSTM_PAST_STEPS + t]);
        assert(host[t] > 0.0f && host[t] < 1.0f);
        mae += fabs((double) host[t] - (double) truth[t]);
    }
    assert(mae / LSTM_FUTURE_STEPS < 0.1);   // a forecast, not noise
    printf("test_mock_soil: self-test window OK (host MAE %.4f)\n", mae / LSTM_FUTURE_STEPS);
}

int main(void) {
    test_mapping();
    test_fill();
    test_lstm_handoff();
    test_selftest_window();
    printf("test_mock_soil: OK\n");
    return 0;
}
