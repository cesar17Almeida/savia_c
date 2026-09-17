// Local storage: an in-RAM ring of recent readings + predictions. Pure logic, so
// the query/aggregation paths unit-test on the host. Feeds the BLE data_request
// handlers (raw / agg / pred + count).
//
// The ring lives in RAM and a reset loses it, so storage_store.c keeps a periodic
// IMAGE of it in flash (see that file). The serialization halves below are here,
// SDK-free, so they stay host-testable: storage_store.c only moves bytes.
//
// Range convention: [from_ms, to_ms) -- from inclusive, to exclusive. Use 0 for
// from and UINT64_MAX for to to mean "no bound". limit 0 = no limit.
#ifndef SAVIA_STORAGE_H
#define SAVIA_STORAGE_H

#include "savia/types.h"
#include <stdbool.h>
#include <stddef.h>

// How many readings the ring holds before the oldest is dropped.
#define SAVIA_READINGS_CAP 600

// One reading on the persistence wire: ts(8) + port(1) + depth(1) + kind(1) + value(4).
// Serialized field by field rather than memcpy'd, so struct padding can never
// change what a stored image means.
#define SAVIA_READING_WIRE_BYTES 15

// Largest blob storage_serialize can produce (u16 count + the full ring).
#define SAVIA_STORAGE_BLOB_MAX (2 + SAVIA_READINGS_CAP * SAVIA_READING_WIRE_BYTES)

void storage_init(void);
void storage_clear(void);   // wipe all readings + predictions (dev "clear data")

// --- Persistence halves (SDK-free; the flash side is storage_store.c) --------

/** Ordered access for a serializer: index 0 is the OLDEST reading. Lets the flash
 *  side stream the ring out a page at a time instead of staging a ~9 KB copy of
 *  it in RAM, which on a board whose BSS is already 430 KB of 520 KB is the
 *  difference worth having. Returns NULL past the end. */
size_t storage_reading_count(void);
const savia_reading_t *storage_reading_at(size_t i);

/** Encode one reading into [SAVIA_READING_WIRE_BYTES] little-endian bytes. */
void storage_encode_reading(const savia_reading_t *r, uint8_t *out);

/** Serialize the whole ring, oldest first (header + every reading). Built on the
 *  three calls above; used by the host tests to round-trip against
 *  storage_restore. The flash side streams instead. Returns bytes written, or 0
 *  if `cap` is too small. */
size_t storage_serialize(uint8_t *out, size_t cap);

/** Replace the ring with a serialized image. Rejects a malformed or over-long
 *  blob rather than loading half of it. Returns how many readings were restored. */
size_t storage_restore(const uint8_t *data, size_t len);   // drops provisional rows

/** True once (and clears) if the ring changed since the last call. Lets the
 *  supervisor skip a flash write when nothing new was captured. */
bool storage_take_dirty(void);

// Any ts below this is uptime-relative ("provisional": captured before the clock
// was synced), never a real epoch (threshold ~ March 1973). See storage_rebase_provisional.
#define SAVIA_TS_PROVISIONAL_MAX 100000000000ULL

// --- Readings ---
bool   storage_append_reading(const savia_reading_t *r);

// Back-fill provisional readings after the first clock sync of this power cycle:
// adds `delta_ms` (= epoch - uptime at sync) to every reading with a provisional
// ts, turning capture-time uptime stamps into the wall time clock_now() would
// have produced. Returns how many readings were rebased.
size_t storage_rebase_provisional(uint64_t delta_ms);

// Drop every reading of one logical port, keeping the rest in FIFO order. Called
// when the installer deletes a sensor and chooses not to keep its data: the port
// is about to be free for a different sensor, and readings are keyed by port.
// Returns how many were removed.
size_t storage_clear_port(uint8_t port);

// Upsert by identity (ts_ms, port, kind, depth_cm): if a matching reading exists
// its value is overwritten and *created=false; otherwise the reading is appended
// and *created=true. Lets the app push/correct points by timestamp ("ingest").
bool   storage_upsert_reading(const savia_reading_t *r, bool *created);
size_t storage_query_raw(uint64_t from_ms, uint64_t to_ms, size_t limit,
                         savia_reading_t *out, size_t out_cap);
size_t storage_count_raw(uint64_t from_ms, uint64_t to_ms);

// Hourly aggregates grouped by (hour, port, kind, depth_cm).
size_t storage_aggregate_hourly(uint64_t from_ms, uint64_t to_ms, size_t limit,
                                savia_aggregate_t *out, size_t out_cap);

// --- Predictions (empty on Pico WH: inference is off-device) ---
bool   storage_append_prediction(const savia_prediction_t *p);
void   storage_clear_predictions(void);   // wipe only predictions (dev: re-mock the forecast)
size_t storage_query_pred(uint64_t from_ms, uint64_t to_ms, size_t limit,
                          savia_prediction_t *out, size_t out_cap);
size_t storage_count_pred(uint64_t from_ms, uint64_t to_ms);

#endif // SAVIA_STORAGE_H
