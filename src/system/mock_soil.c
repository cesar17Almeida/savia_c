#include "savia/mock_soil.h"
#include "savia/mock_soil_data.h"
#include "savia/storage.h"
#include "savia/types.h"

#define HOUR_MS   3600000ULL
#define MINUTE_MS 60000LL
#define WINDOW_H  (MOCK_SOIL_PAST_H + MOCK_SOIL_AHEAD_H)

_Static_assert(MOCK_SOIL_ROWS % 24 == 0, "the replay must hold whole days");
_Static_assert(MOCK_SELFTEST_ROW + LSTM_PAST_STEPS + LSTM_FUTURE_STEPS <= MOCK_SOIL_ROWS,
               "self-test window outside the replay");

static const uint8_t DEPTHS[2] = { 10, 30 };

// Local hours since the epoch, wrapped onto the replay; whole days keep the hour of day.
static uint32_t replay_row(uint64_t ts_ms, int16_t utc_offset_min) {
    int64_t local = (int64_t) ts_ms + (int64_t) utc_offset_min * MINUTE_MS;
    if (local < 0) local = 0;
    return (uint32_t) (((uint64_t) local / HOUR_MS) % MOCK_SOIL_ROWS);
}

bool mock_soil_value(uint64_t ts_ms, int16_t utc_offset_min, uint8_t depth_cm, float *out) {
    uint32_t row = replay_row(ts_ms, utc_offset_min);
    if (depth_cm == 10)      *out = MOCK_SOIL_HS10[row];
    else if (depth_cm == 30) *out = MOCK_SOIL_HS30[row];
    else return false;
    return true;
}

size_t mock_soil_fill(uint64_t now_ms, int16_t utc_offset_min) {
    if (now_ms < SAVIA_TS_PROVISIONAL_MAX) return 0;   // no wall clock yet
    uint64_t first = now_ms - now_ms % HOUR_MS - (uint64_t) (MOCK_SOIL_PAST_H - 1) * HOUR_MS;

    // One pass over the ring marks the (hour, depth) slots that already hold data.
    bool have[WINDOW_H][2] = { { false } };
    size_t n = storage_reading_count();
    for (size_t i = 0; i < n; i++) {
        const savia_reading_t *r = storage_reading_at(i);
        if (r->port != MOCK_SOIL_PORT || r->kind != READING_SOIL_MOISTURE || r->ts_ms < first)
            continue;
        uint64_t k = (r->ts_ms - first) / HOUR_MS;
        if (k >= WINDOW_H) continue;
        for (int d = 0; d < 2; d++)
            if (r->depth_cm == DEPTHS[d]) have[k][d] = true;
    }

    size_t added = 0;
    for (uint32_t k = 0; k < WINDOW_H; k++) {           // oldest first, like real captures
        for (int d = 0; d < 2; d++) {
            if (have[k][d]) continue;
            savia_reading_t r = { .ts_ms = first + (uint64_t) k * HOUR_MS, .port = MOCK_SOIL_PORT,
                                  .depth_cm = DEPTHS[d], .kind = READING_SOIL_MOISTURE };
            mock_soil_value(r.ts_ms, utc_offset_min, r.depth_cm, &r.value);
            storage_append_reading(&r);
            added++;
        }
    }
    return added;
}

void mock_soil_selftest_window(lstm_raw_inputs_t *raw, const float **host,
                               const float **truth) {
    for (int t = 0; t < LSTM_PAST_STEPS; t++) {
        raw->ta[t]   = MOCK_SELFTEST_TA[t];
        raw->hs10[t] = MOCK_SOIL_HS10[MOCK_SELFTEST_ROW + t];
        raw->hs30[t] = MOCK_SOIL_HS30[MOCK_SELFTEST_ROW + t];
    }
    for (int t = 0; t < LSTM_FUTURE_STEPS; t++)
        raw->future_ta[t] = MOCK_SELFTEST_TA[LSTM_PAST_STEPS + t];
    *host = MOCK_SELFTEST_HOST;
    *truth = &MOCK_SOIL_HS30[MOCK_SELFTEST_ROW + LSTM_PAST_STEPS];
}
