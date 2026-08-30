// Weather cache: the air-temperature (TA) forecast the LSTM needs, delivered by
// the LoRa downlink (or, when a phone is around, over BLE). Mirrors savia_py's
// `state.set_weather` -- past + future hourly TA, plus when it was last updated.
// Pure logic (no SDK), so it is host-testable and shared by both boards.
#ifndef SAVIA_WEATHER_H
#define SAVIA_WEATHER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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

// --- Persistence halves (SDK-free; the flash side is storage_store.c) --------
// The TA window rides in the same flash record as the readings ring, because the
// model needs BOTH to infer and losing either one costs the same 24 h of
// re-sampling. 298 B, so it is free next to the ring's ~9 KB.

// u8 n_past + u8 n_future + u64 updated_ms + the values.
#define WEATHER_BLOB_MAX (2 + 8 + (WEATHER_PAST_MAX + WEATHER_FUTURE_MAX) * 4)

/** Serialize the cache. Returns bytes written, or 0 if unset or `cap` too small. */
size_t weather_serialize(uint8_t *out, size_t cap);

/**
 * Load a serialized cache, but ONLY if it is still aligned to the present.
 *
 * The window is positional, not timestamped per value: its newest past sample
 * means "the hour this was set in". Restoring a window from three days ago would
 * hand the model air temperatures for the wrong hours and it would never notice
 * -- lstm_gather_inputs checks that the cache is full, not that it is current.
 * So a restored window older than [max_age_ms] against `now_ms` is dropped and
 * the station simply waits for the next downlink.
 *
 * Returns true if the cache was adopted.
 */
bool weather_restore(const uint8_t *data, size_t len, uint64_t now_ms, uint64_t max_age_ms);

/** How stale a restored TA window may be. One hour: the window is aligned to
 *  whole hours, so past that it is describing a different hour than the one the
 *  model would be inferring. */
#define WEATHER_RESTORE_MAX_AGE_MS (3600000ULL)

#endif // SAVIA_WEATHER_H
