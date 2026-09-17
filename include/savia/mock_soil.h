// Mock soil probe (dev): replays measured hourly HS10/HS30 from the LSTM dataset
// (see mock_soil_data.h) as port-1 readings. Only what the probe measures; air
// temperature keeps arriving as a forecast (LoRa downlink or BLE weather).
// Pure logic on top of storage, so it host-tests.
#ifndef SAVIA_MOCK_SOIL_H
#define SAVIA_MOCK_SOIL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "savia/lstm_input.h"

#define MOCK_SOIL_PORT    1
#define MOCK_SOIL_PAST_H  LSTM_PAST_STEPS     // model history, ending at the current hour
#define MOCK_SOIL_AHEAD_H LSTM_OUTPUT_STEPS   // forecast horizon, kept as truth to score it

// Replayed value of `depth_cm` (10 or 30) for the hour holding `ts_ms`. The row
// depends only on that LOCAL hour, so an hour always reads the same value.
bool mock_soil_value(uint64_t ts_ms, int16_t utc_offset_min, uint8_t depth_cm, float *out);

// Append the replayed readings missing from [hour - 47 h, hour + 24 h], where
// hour holds `now_ms` (a wall clock). An hour that already has a reading of that
// depth on port 1 keeps it. Returns how many readings were appended.
size_t mock_soil_fill(uint64_t now_ms, int16_t utc_offset_min);

// Boot self-test input: one real 72 h window of the replay with its own air
// temperature, plus the host forecast and the measured HS30 (24 values each).
void mock_soil_selftest_window(lstm_raw_inputs_t *raw, const float **host,
                               const float **truth);

#endif // SAVIA_MOCK_SOIL_H
