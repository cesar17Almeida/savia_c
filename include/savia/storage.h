// Local storage: a circular log in on-chip flash (replaces SQLite from savia_py).
// The data is tiny (hourly aggregates over >=48 h), so a wear-levelled ring of
// fixed-size records is enough; the BLE handlers query it for raw/agg/pred.
#ifndef SAVIA_STORAGE_H
#define SAVIA_STORAGE_H

#include "savia/types.h"
#include <stdbool.h>

void storage_init(void);

// Append one reading to the ring. Returns false if the flash write failed.
bool storage_append_reading(const savia_reading_t *r);

// Range query feeding the BLE "raw" kind. Returns count written (<= max).
int storage_query_raw(uint64_t from_ms, uint64_t to_ms,
                      savia_reading_t *out, int max);

#endif // SAVIA_STORAGE_H
