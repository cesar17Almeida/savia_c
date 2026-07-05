// Host-side unit test for the clock module: the raw running clock plus the
// authoritative-sync ring, validation, and outage measurement. Compiles NATIVELY
// (clock.c is SDK-free). Flash I/O (clock_store.c) is firmware-only and not tested
// here -- this covers the pure logic the store persists.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "savia/clock.h"

// 2025-01-01T00:00:00Z, comfortably inside the plausibility window.
#define BASE 1735689600000ULL
#define H    3600000ULL

// Build a persisted-ring blob with a single sample (what "flash from before the
// reboot" would look like), so we can simulate a power cycle within one process.
static size_t make_seed_blob(uint64_t epoch, uint8_t src, uint8_t *out) {
    size_t p = 0;
    out[p++] = 1;                               // count
    for (int i = 0; i < 8; i++) out[p++] = (uint8_t)(epoch >> (8 * i));
    out[p++] = src;
    return p;
}

int main(void) {
    // 1) Cold start: nothing synced yet.
    assert(!clock_is_set());
    assert(clock_now(1000) == 0);
    assert(clock_last_known() == 0);
    assert(clock_boot_outage_ms() == 0);

    // 2) Empty ring serializes to a single count byte.
    uint8_t buf[CLOCK_RING_BLOB_MAX];
    assert(clock_serialize_ring(buf, sizeof buf) == 1 && buf[0] == 0);

    // 3) Seed the ring from "flash" (pre-outage reference). Does NOT set the clock.
    uint8_t seed[CLOCK_RING_BLOB_MAX];
    size_t sl = make_seed_blob(BASE, CLOCK_SRC_LORA, seed);
    assert(clock_seed_ring(seed, sl));
    assert(clock_last_known() == BASE);
    assert(!clock_is_set());                    // seeding is history, not "now"

    // 4) First post-boot sync 2 h later -> measures the outage and sets the clock.
    uint64_t outage = 0;
    assert(clock_apply_sync(BASE + 2 * H, 1000, CLOCK_SRC_LORA, &outage));
    assert(outage == 2 * H);
    assert(clock_boot_outage_ms() == 2 * H);
    assert(clock_is_set());
    assert(clock_now(1000) == BASE + 2 * H);
    assert(clock_now(2000) == BASE + 2 * H + 1000);   // +1 s uptime
    assert(clock_last_known() == BASE + 2 * H);
    assert(clock_last_sync_ms() == BASE + 2 * H);
    assert(clock_take_ring_dirty());            // an accepted sync dirties the ring
    assert(!clock_take_ring_dirty());           // ...and clears on read

    // 5) Reject implausible epochs; state unchanged.
    assert(!clock_apply_sync(0, 2000, CLOCK_SRC_LORA, NULL));
    assert(!clock_apply_sync(CLOCK_EPOCH_MIN_MS - 1, 2000, CLOCK_SRC_LORA, NULL));
    assert(!clock_apply_sync(CLOCK_EPOCH_MAX_MS, 2000, CLOCK_SRC_LORA, NULL));
    assert(clock_last_known() == BASE + 2 * H);
    assert(!clock_take_ring_dirty());           // rejects don't dirty

    // 6) Reject a backward jump past the slack.
    assert(!clock_apply_sync(BASE + H, 2000, CLOCK_SRC_BLE, NULL));
    assert(clock_last_known() == BASE + 2 * H);

    // 7) Accept the next hourly sync (forward). boot_outage stays the first value.
    assert(clock_apply_sync(BASE + 3 * H, 1000 + H, CLOCK_SRC_LORA, &outage));
    assert(outage == H);                        // gap vs the previous known-good
    assert(clock_boot_outage_ms() == 2 * H);    // unchanged: only the first sync
    assert(clock_last_known() == BASE + 3 * H);

    // 8) Ring keeps only the last CLOCK_RING_MAX, newest first.
    assert(clock_apply_sync(BASE + 4 * H, 1000 + 2 * H, CLOCK_SRC_LORA, NULL));
    clock_sample_t r[CLOCK_RING_MAX];
    uint8_t n = clock_get_ring(r, CLOCK_RING_MAX);
    assert(n == CLOCK_RING_MAX);
    assert(r[0].epoch_ms == BASE + 4 * H);
    assert(r[1].epoch_ms == BASE + 3 * H);
    assert(r[2].epoch_ms == BASE + 2 * H);      // BASE seed fell off the end

    // 9) Small backward jitter within the slack is tolerated.
    assert(clock_apply_sync(BASE + 4 * H - (CLOCK_BACKWARD_SLACK_MS - 1), 1000 + 3 * H,
                            CLOCK_SRC_BLE, NULL));
    // ...but just past the slack is rejected.
    uint64_t head = clock_last_known();
    assert(!clock_apply_sync(head - CLOCK_BACKWARD_SLACK_MS - 1, 1, CLOCK_SRC_BLE, NULL));

    // 10) Serialize -> seed round-trip preserves head + count.
    uint8_t blob[CLOCK_RING_BLOB_MAX];
    size_t bl = clock_serialize_ring(blob, sizeof blob);
    assert(bl == (size_t) 1 + CLOCK_RING_MAX * 9);
    uint64_t saved_head = clock_last_known();
    assert(clock_seed_ring(blob, bl));
    assert(clock_last_known() == saved_head);
    clock_sample_t r2[CLOCK_RING_MAX];
    assert(clock_get_ring(r2, CLOCK_RING_MAX) == CLOCK_RING_MAX);

    // 11) Malformed blobs are rejected (ring emptied).
    assert(!clock_seed_ring(NULL, 0));
    uint8_t bad[2] = { 99, 0 };                 // count > CLOCK_RING_MAX
    assert(!clock_seed_ring(bad, sizeof bad));
    assert(clock_last_known() == 0);

    printf("test_clock: running clock + ring + validation + outage OK\n");
    return 0;
}
