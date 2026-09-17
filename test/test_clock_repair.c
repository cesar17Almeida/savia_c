// Host test: a LoRa time may repair a clock pushed into the future, but a late
// downlink may not rewind a correct one. Its own program so it starts from a
// clock that has never run (the cold-boot case needs that).
#include <assert.h>
#include <stdio.h>
#include "savia/clock.h"

#define BASE 1735689600000ULL   // 2025-01-01T00:00:00Z
#define H    3600000ULL
#define YEAR (8760 * H)
#define P    3600u              // LoRa cadence under test: 1 h

static size_t seed_blob(uint64_t epoch, uint8_t *out) {
    size_t p = 0;
    out[p++] = 1;
    for (int i = 0; i < 8; i++) out[p++] = (uint8_t)(epoch >> (8 * i));
    out[p++] = CLOCK_SRC_BLE;
    return p;
}

int main(void) {
    // Staleness bound: two periods plus a margin, never under an hour.
    assert(clock_lora_stale_max_ms(P) == 2 * H + 600000ULL);
    assert(clock_lora_stale_max_ms(300) == H);
    assert(clock_lora_stale_max_ms(86400) == 48 * H + 600000ULL);

    // --- cold boot with a poisoned ring in flash: no running clock ---
    {
        uint8_t blob[CLOCK_RING_BLOB_MAX];
        assert(clock_seed_ring(blob, seed_blob(BASE + 5 * YEAR, blob)));
        uint64_t real = BASE;
        // The BOOT downlink is far behind the restored reference: held, clock still off.
        assert(clock_apply_sync_lora(real, 5000, P, NULL) == CLOCK_SYNC_HELD);
        assert(!clock_is_set() && clock_repair_pending());
        // Six hours later a second one agrees (against uptime): the clock starts.
        uint64_t out = 1;
        assert(clock_apply_sync_lora(real + 6 * H + 2000, 5000 + 6 * H, P, &out)
               == CLOCK_SYNC_REPAIRED);
        assert(clock_is_set() && !clock_repair_pending() && out == 0);
        assert(clock_now(5000 + 6 * H) == real + 6 * H + 2000);
        clock_sample_t ring[CLOCK_RING_MAX];
        assert(clock_get_ring(ring, CLOCK_RING_MAX) == 1);   // poisoned history dropped
        uint64_t back = 1;
        assert(clock_take_step_back(&back) && back == 0);    // only restored rows are ahead
        assert(!clock_take_step_back(NULL));
        printf("test_clock_repair: cold boot with a poisoned ring OK\n");
    }

    const uint64_t U0 = 100 * H;             // uptime when real time is t0
    const uint64_t t0 = BASE + 200 * H;
    assert(clock_apply_sync_lora(t0, U0, P, NULL) == CLOCK_SYNC_APPLIED);
    assert(!clock_take_step_back(NULL));     // forward: nothing to rewind

    // --- a correct clock is never rewound by late downlinks ---
    {
        // One and two periods late, twice in a row: refused, nothing held.
        assert(clock_apply_sync_lora(t0 - H, U0 + H, P, NULL) == CLOCK_SYNC_REJECTED);
        assert(clock_apply_sync_lora(t0 - H + 3000, U0 + H + 3000, P, NULL) == CLOCK_SYNC_REJECTED);
        assert(clock_apply_sync_lora(t0 - 2 * H, U0 + H, P, NULL) == CLOCK_SYNC_REJECTED);
        assert(!clock_repair_pending());
        assert(clock_now(U0 + H) == t0 + H);
        printf("test_clock_repair: late downlinks refused OK\n");
    }

    // --- running clock pushed years ahead by an unauthenticated write ---
    uint64_t poison = t0 + 2 * H + 5 * YEAR;
    const uint64_t U1 = U0 + 2 * H;
    {
        assert(clock_apply_sync(poison, U1, CLOCK_SRC_BLE, NULL));   // forward: accepted
        assert(!clock_take_step_back(NULL));
        // The plain rule refuses every honest time from now on.
        assert(!clock_apply_sync(t0 + 3 * H, U1 + H, CLOCK_SRC_LORA, NULL));
        // Unauthenticated BLE never gets the LoRa path, however often it writes.
        assert(!clock_apply_sync(t0 + 3 * H, U1 + H, CLOCK_SRC_BLE, NULL));
        assert(!clock_apply_sync(t0 + 3 * H, U1 + H, CLOCK_SRC_BLE, NULL));
        assert(!clock_repair_pending());

        uint64_t out = 1;
        assert(clock_apply_sync_lora(t0 + 3 * H, U1 + H, P, &out) == CLOCK_SYNC_HELD);
        assert(clock_repair_pending() && out == 0);
        assert(clock_now(U1 + H) == poison + H);                     // held, not applied
        // Six hours later, 3 s off: agrees.
        uint64_t fix = t0 + 9 * H + 3000;
        assert(clock_apply_sync_lora(fix, U1 + 7 * H, P, &out) == CLOCK_SYNC_REPAIRED);
        assert(!clock_repair_pending() && out == 0);
        assert(clock_now(U1 + 7 * H) == fix && clock_last_known() == fix);
        clock_sample_t ring[CLOCK_RING_MAX];
        assert(clock_get_ring(ring, CLOCK_RING_MAX) == 1);
        uint64_t back = 0;
        assert(clock_take_step_back(&back) && back == poison + 7 * H - fix);
        assert(!clock_take_step_back(NULL));
        assert(clock_apply_sync_lora(fix + H, U1 + 8 * H, P, NULL) == CLOCK_SYNC_APPLIED);
        printf("test_clock_repair: poisoned running clock repaired by two LoRa times OK\n");
    }

    // --- held times must agree; a plausible one cancels the hold ---
    {
        const uint64_t U2 = U1 + 10 * H;             // real time t0 + 12 h
        poison = t0 + 12 * H + YEAR;
        assert(clock_apply_sync(poison, U2, CLOCK_SRC_BLE, NULL));
        // A day-old downlink, then a fresh one: they disagree, the newer is held.
        assert(clock_apply_sync_lora(t0 + 13 * H - 24 * H, U2 + H, P, NULL) == CLOCK_SYNC_HELD);
        assert(clock_apply_sync_lora(t0 + 14 * H, U2 + 2 * H, P, NULL) == CLOCK_SYNC_HELD);
        assert(clock_now(U2 + 2 * H) == poison + 2 * H);
        // A time the clock can explain as late drops the hold.
        assert(clock_apply_sync_lora(poison - H, U2 + 2 * H, P, NULL) == CLOCK_SYNC_REJECTED);
        assert(!clock_repair_pending());
        // Tolerance: 6 min apart does not agree, 5 min does.
        assert(clock_apply_sync_lora(t0 + 15 * H, U2 + 3 * H, P, NULL) == CLOCK_SYNC_HELD);
        assert(clock_apply_sync_lora(t0 + 16 * H + 6 * 60000ULL, U2 + 4 * H, P, NULL)
               == CLOCK_SYNC_HELD);
        assert(clock_apply_sync_lora(t0 + 17 * H + 11 * 60000ULL, U2 + 5 * H, P, NULL)
               == CLOCK_SYNC_REPAIRED);
        assert(clock_take_step_back(NULL));
        printf("test_clock_repair: disagreeing and cancelled holds OK\n");
    }

    // --- an authenticated owner rewinds at once and flags it too ---
    {
        const uint64_t U3 = U1 + 20 * H;
        uint64_t real = t0 + 22 * H + 11 * 60000ULL;   // continues the repaired timeline
        assert(clock_apply_sync(real + YEAR, U3, CLOCK_SRC_BLE, NULL));
        assert(clock_apply_sync_trusted(real, U3, CLOCK_SRC_BLE, NULL));
        uint64_t back = 0;
        assert(clock_take_step_back(&back) && back == YEAR);
        // A trusted time a few seconds behind is not worth moving readings for.
        assert(clock_apply_sync_trusted(real - 6000, U3, CLOCK_SRC_BLE, NULL));
        assert(!clock_take_step_back(NULL));
        printf("test_clock_repair: trusted rewind flagged OK\n");
    }

    printf("test_clock_repair: OK\n");
    return 0;
}
