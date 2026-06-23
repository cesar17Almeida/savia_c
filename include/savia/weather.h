// Weather cache: the air-temperature (TA) forecast the LSTM needs, delivered by
// the LoRa downlink (or, when a phone is around, over BLE). Mirrors savia_py's
// `state.set_weather` -- past + future hourly TA, plus when it was last updated.
// Pure logic (no SDK), so it is host-testable and shared by both boards.
#ifndef SAVIA_WEATHER_H
#define SAVIA_WEATHER_H

#include <stdint.h>
#include <stdbool.h>

// Bounds match the LSTM window (48 past + 24 future hourly TA values).
#define WEATHER_PAST_MAX     48
#define WEATHER_FUTURE_MAX   24

// Replace the cached forecast. Counts are clamped to the maxes above.
// `updated_ms` is the epoch ms the forecast was applied (0 if the clock is unset).
void weather_set(const float *past_ta, uint8_t n_past,
                 const float *future_ta, uint8_t n_future, uint64_t updated_ms);

// Accessors. *out points at the internal array (valid until the next set);
// the return value is the count. Pass NULL for *out to just read the count.
uint8_t  weather_get_past(const float **out);
uint8_t  weather_get_future(const float **out);
uint64_t weather_updated_ms(void);   // 0 until the first set with a known clock
bool     weather_is_set(void);

#endif // SAVIA_WEATHER_H
