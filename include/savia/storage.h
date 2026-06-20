// Local storage: an in-RAM ring of recent readings + predictions (a flash-backed
// ring comes later). Pure logic, so the query/aggregation paths unit-test on the
// host. Feeds the BLE data_request handlers (raw / agg / pred + count).
//
// Range convention: [from_ms, to_ms) -- from inclusive, to exclusive. Use 0 for
// from and UINT64_MAX for to to mean "no bound". limit 0 = no limit.
#ifndef SAVIA_STORAGE_H
#define SAVIA_STORAGE_H

#include "savia/types.h"
#include <stdbool.h>
#include <stddef.h>

void storage_init(void);

// --- Readings ---
bool   storage_append_reading(const savia_reading_t *r);
size_t storage_query_raw(uint64_t from_ms, uint64_t to_ms, size_t limit,
                         savia_reading_t *out, size_t out_cap);
size_t storage_count_raw(uint64_t from_ms, uint64_t to_ms);

// Hourly aggregates grouped by (hour, port, kind, depth_cm).
size_t storage_aggregate_hourly(uint64_t from_ms, uint64_t to_ms, size_t limit,
                                savia_aggregate_t *out, size_t out_cap);

// --- Predictions (empty on Pico WH: inference is off-device) ---
bool   storage_append_prediction(const savia_prediction_t *p);
size_t storage_query_pred(uint64_t from_ms, uint64_t to_ms, size_t limit,
                          savia_prediction_t *out, size_t out_cap);
size_t storage_count_pred(uint64_t from_ms, uint64_t to_ms);

#endif // SAVIA_STORAGE_H
