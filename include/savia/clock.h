// Software wall clock + authoritative-sync history. The Pico has no battery-backed
// RTC, so time comes from the app (BLE) or a LoRa downlink; we keep an epoch base
// relative to the board uptime. On top of the raw clock, a small persisted ring of
// the last few syncs lets a reboot measure how long the board was powered off (the
// outage): there is NO on-device way to time an outage, so we compare a fresh sync
// against the last-known-good saved before the power loss. Pure logic (uptime and
// bytes passed in) so it unit-tests on the host; flash I/O lives in clock_store.c.
#ifndef SAVIA_CLOCK_H
#define SAVIA_CLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Where a sync came from (kept in the ring for diagnostics / cross-checking).
typedef enum {
    CLOCK_SRC_NONE = 0,
    CLOCK_SRC_BLE  = 1,   // phone time_sync over BLE
    CLOCK_SRC_LORA = 2,   // TTN downlink clock field
    CLOCK_SRC_AON  = 3,   // always-on timer across a deep sleep (derived, not a sync)
} clock_source_t;

// How many authoritative syncs we keep (the user asked for "the last 2-3").
#define CLOCK_RING_MAX 3

// Plausibility window: reject any epoch outside [2024-01-01 .. 2100-01-01). Guards
// against a garbage LoRa downlink (0, 0xFFFFFFFF, ...) poisoning the clock.
#define CLOCK_EPOCH_MIN_MS 1704067200000ULL   // 2024-01-01T00:00:00Z
#define CLOCK_EPOCH_MAX_MS 4102444800000ULL   // 2100-01-01T00:00:00Z
// Tolerate a few seconds of backward jitter between sources (second-resolution
// rounding); a larger backward jump vs the last known-good is rejected.
#define CLOCK_BACKWARD_SLACK_MS 5000ULL
// An outage above this is worth a warning and marks a data gap (1 h skew is
// agronomically tolerable per Antonio).
#define CLOCK_OUTAGE_WARN_MS 3600000ULL
// A queued LoRa downlink reaches the node one uplink late, two if one was lost:
// a backward LoRa time within 2 periods + this margin (never under the floor)
// is taken as stale. Further back, it is held until a second one agrees with it
// to within the tolerance, and then it repairs the clock.
#define CLOCK_LORA_STALE_MARGIN_MS  600000ULL    // 10 min
#define CLOCK_LORA_STALE_MIN_MS     3600000ULL   // 1 h
#define CLOCK_CONFIRM_TOLERANCE_MS  300000ULL    // 5 min
// A backward step this small (a LoRa time lands ~6 s behind a BLE one) is not
// worth moving stored readings for.
#define CLOCK_STEP_BACK_MIN_MS      60000ULL

typedef enum {
    CLOCK_SYNC_REJECTED = 0,
    CLOCK_SYNC_APPLIED,
    CLOCK_SYNC_HELD,       // far behind the clock: waiting for a second LoRa time
    CLOCK_SYNC_REPAIRED,   // agreed with the held one: the clock moved back
} clock_sync_result_t;

// One recorded sync. uptime_ms is only meaningful within a power cycle (0 after a
// reboot-seed) -- across an outage it cannot be related to the new uptime.
typedef struct {
    uint64_t epoch_ms;
    uint64_t uptime_ms;   // ms-since-boot when applied (0 when seeded from flash)
    uint8_t  source;      // clock_source_t
} clock_sample_t;

// --- raw running clock (unchanged contract; used by tests and the status read) --

// Set the wall clock: `epoch_ms` is "now", `uptime_ms` the current ms-since-boot.
// Unconditional (no validation, no ring) -- clock_apply_sync() calls this after it
// accepts a sync; direct use is fine where validation isn't wanted.
void clock_set(uint64_t epoch_ms, uint64_t uptime_ms);

// Current epoch ms, given the current uptime. Returns 0 if never synced.
uint64_t clock_now(uint64_t uptime_ms);

// True once the running clock has been set at least once this power cycle.
bool clock_is_set(void);

// Epoch ms of the last running-clock set (0 if never). For the status payload.
uint64_t clock_last_sync_ms(void);

// --- authoritative sync + outage recovery -----------------------------------

// Apply an authoritative time with validation. Rejects (returns false, no state
// change) if epoch is out of the plausibility window or jumps backward past the
// slack vs the last known-good. On accept: pushes the ring, sets the running clock,
// marks the ring dirty, and returns true. `*outage_ms` (if non-NULL) receives the
// gap vs the previous last-known-good (0 if none) -- on the FIRST sync after a
// reboot this is the measured power-off duration.
bool clock_apply_sync(uint64_t epoch_ms, uint64_t uptime_ms, clock_source_t source,
                      uint64_t *outage_ms);

// Same, for an authenticated owner: may also move the clock backwards, discarding
// the samples ahead of it, so one bogus future time cannot block every later sync.
bool clock_apply_sync_trusted(uint64_t epoch_ms, uint64_t uptime_ms, clock_source_t source,
                              uint64_t *outage_ms);

// A LoRa downlink time. Authentic, but possibly late: see CLOCK_LORA_STALE_*.
// `period_s` is the uplink cadence, which bounds how late it can be.
clock_sync_result_t clock_apply_sync_lora(uint64_t epoch_ms, uint64_t uptime_ms,
                                          uint32_t period_s, uint64_t *outage_ms);

// How far behind the last known-good a LoRa time may be and still count as late.
uint64_t clock_lora_stale_max_ms(uint32_t period_s);

// True while a far-backward LoRa time waits for confirmation (RAM only: the
// supervisor stays out of deep sleep meanwhile).
bool clock_repair_pending(void);

// True (clearing it) if a correction (trusted, or LoRa-repaired) moved time
// backwards since the last call. `*delta_ms` is how far the running clock moved
// (0 if it was not running: only history restored from flash is ahead).
bool clock_take_step_back(uint64_t *delta_ms);

// Continue the clock after a deep sleep from the always-on timer that kept
// counting while the core was off. Sets the running clock and the ring head
// (source AON) like an accepted sync, but does NOT mark the ring dirty: it is a
// derived value, not an authority worth a flash write. `uncertainty_ms` is the
// drift the low-power oscillator may have accumulated: the next real sync is
// accepted even if it lands that far behind the estimate, and clears it. Rejects
// (false) an epoch outside the plausibility window or behind the last known-good.
bool clock_resume_from_aon(uint64_t epoch_ms, uint64_t uptime_ms, uint32_t uncertainty_ms);

// Drift allowance currently added to the backward slack (0 after a real sync).
uint32_t clock_uncertainty_ms(void);

// Newest known-good epoch across power cycles (ring head), or 0 if the ring is
// empty. After clock_seed_ring() at boot this is the pre-outage reference.
uint64_t clock_last_known(void);

// The outage measured at the first post-boot sync (0 until then / if unknown).
uint64_t clock_boot_outage_ms(void);

// Copy up to `max` ring samples, newest first. Returns the count copied.
uint8_t clock_get_ring(clock_sample_t *out, uint8_t max);

// --- persistence bridge (bytes only; flash lives in clock_store.c) -----------

// Serialize the ring into `out`. Returns bytes written, or 0 if `cap` is too small.
size_t clock_serialize_ring(uint8_t *out, size_t cap);

// Load the ring from bytes (as written by clock_serialize_ring) WITHOUT setting the
// running clock -- time has passed during the outage. Returns false on a malformed
// buffer (ring left empty). uptime_ms of seeded samples is 0.
bool clock_seed_ring(const uint8_t *data, size_t len);

// True (clearing the flag) if the ring changed since the last check -- the
// supervisor uses this to know when to re-persist via clock_store_save().
bool clock_take_ring_dirty(void);

// Largest buffer clock_serialize_ring can produce (header + full ring).
#define CLOCK_RING_BLOB_MAX (1 + CLOCK_RING_MAX * (8 + 1))

// --- flash persistence (clock_store.c -- firmware only, uses the Pico SDK) ----

// Read the persisted ring from flash and seed it (call at boot, before BLE starts).
// Returns true if a valid record was restored. Host builds don't link this.
bool clock_store_load(void);
// Write a ring blob (from clock_serialize_ring, snapshotted under the BLE lock by
// the caller) to the clock flash sector. Kept separate from serialize so the flash
// write never races a BLE-context sync.
void clock_store_save(const uint8_t *blob, size_t len);

#endif // SAVIA_CLOCK_H
