// The persistence halves that storage_store.c moves to flash: serializing the
// readings ring, restoring it, tracking whether anything changed, and the TA
// window's freshness rule. All SDK-free, so it runs on the host.
//
// What these guard is the difference between "the station came back with its
// history" and "the station came back with a plausible-looking wrong history",
// which is the failure mode that matters: nothing downstream re-checks it.
#include "savia/storage.h"
#include "savia/weather.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define HOUR_MS 3600000ULL

static savia_reading_t mk(uint64_t ts, uint8_t port, uint8_t depth, float v) {
    savia_reading_t r = { .ts_ms = ts, .port = port, .depth_cm = depth,
                          .kind = READING_SOIL_MOISTURE, .value = v };
    return r;
}

static void test_roundtrip(void) {
    storage_init();
    const uint64_t base = 1787616000000ULL;
    for (int h = 0; h < 48; h++) {
        savia_reading_t a = mk(base + (uint64_t) h * HOUR_MS, 1, 10, 0.70f + h * 0.001f);
        savia_reading_t b = mk(base + (uint64_t) h * HOUR_MS, 1, 30, 0.74f + h * 0.001f);
        assert(storage_append_reading(&a));
        assert(storage_append_reading(&b));
    }
    static uint8_t blob[SAVIA_STORAGE_BLOB_MAX];
    size_t n = storage_serialize(blob, sizeof blob);
    assert(n == 2 + 96 * SAVIA_READING_WIRE_BYTES);

    // Wipe, then bring it back: every field must survive, in order.
    storage_clear();
    assert(storage_count_raw(0, UINT64_MAX) == 0);
    assert(storage_restore(blob, n) == 96);
    assert(storage_count_raw(0, UINT64_MAX) == 96);

    savia_reading_t out[96];
    assert(storage_query_raw(0, UINT64_MAX, 0, out, 96) == 96);
    for (int i = 0; i < 96; i++) {
        int h = i / 2;
        uint8_t depth = (i % 2) ? 30 : 10;
        assert(out[i].ts_ms == base + (uint64_t) h * HOUR_MS);
        assert(out[i].port == 1);
        assert(out[i].depth_cm == depth);
        assert(out[i].kind == READING_SOIL_MOISTURE);
        float want = (depth == 10 ? 0.70f : 0.74f) + h * 0.001f;
        assert(fabsf(out[i].value - want) < 1e-6f);
    }
    printf("test_storage_persist: round-trip OK (96 readings, exact)\n");
}

static void test_wrapped_ring_keeps_fifo_order(void) {
    // Overflow the ring so head != 0, then round-trip: the image must come back
    // oldest-first, or the restored history is scrambled in time.
    storage_init();
    for (size_t i = 0; i < SAVIA_READINGS_CAP + 37; i++) {
        savia_reading_t r = mk(1000000000000ULL + i * 60000ULL, 1, 10, (float) i);
        storage_append_reading(&r);
    }
    static uint8_t blob[SAVIA_STORAGE_BLOB_MAX];
    size_t n = storage_serialize(blob, sizeof blob);
    storage_clear();
    assert(storage_restore(blob, n) == SAVIA_READINGS_CAP);

    static savia_reading_t out[SAVIA_READINGS_CAP];
    size_t got = storage_query_raw(0, UINT64_MAX, 0, out, SAVIA_READINGS_CAP);
    assert(got == SAVIA_READINGS_CAP);
    // The 37 oldest fell off the ring; what is left is contiguous and ascending.
    assert(fabsf(out[0].value - 37.0f) < 1e-6f);
    for (size_t i = 1; i < got; i++) assert(out[i].ts_ms > out[i - 1].ts_ms);
    printf("test_storage_persist: wrapped ring keeps FIFO order OK\n");
}

static void test_malformed_blobs_are_refused_whole(void) {
    storage_init();
    savia_reading_t r = mk(1787616000000ULL, 1, 10, 0.5f);
    storage_append_reading(&r);
    static uint8_t blob[SAVIA_STORAGE_BLOB_MAX];
    size_t n = storage_serialize(blob, sizeof blob);

    // A truncated image must load NOTHING, not a prefix: a half-restored ring
    // reads as real history with a hole in it.
    assert(storage_restore(blob, n - 1) == 0);
    assert(storage_restore(blob, 1) == 0);
    assert(storage_restore(NULL, n) == 0);

    // A count past the ring's capacity is a different firmware's image.
    uint8_t bad[8] = { 0 };
    bad[0] = (uint8_t) ((SAVIA_READINGS_CAP + 1) & 0xff);
    bad[1] = (uint8_t) ((SAVIA_READINGS_CAP + 1) >> 8);
    assert(storage_restore(bad, sizeof bad) == 0);
    printf("test_storage_persist: malformed images refused OK\n");
}

static void test_dirty_tracks_every_mutation(void) {
    storage_init();
    assert(!storage_take_dirty());              // fresh ring: nothing to write

    savia_reading_t r = mk(1787616000000ULL, 1, 10, 0.5f);
    storage_append_reading(&r);
    assert(storage_take_dirty());
    assert(!storage_take_dirty());              // taking it clears it

    bool created = false;
    r.value = 0.6f;
    storage_upsert_reading(&r, &created);
    assert(!created);                           // overwrote in place
    assert(storage_take_dirty());               // ...and that still needs persisting

    storage_rebase_provisional(1000);           // nothing provisional here
    assert(!storage_take_dirty());              // a no-op must NOT force a flash write

    savia_reading_t prov = mk(45000, 2, 0, 1.0f);
    storage_append_reading(&prov);
    (void) storage_take_dirty();
    assert(storage_rebase_provisional(1000) == 1);
    assert(storage_take_dirty());

    assert(storage_clear_port(2) == 1);
    assert(storage_take_dirty());
    assert(storage_clear_port(9) == 0);         // no such port
    assert(!storage_take_dirty());

    storage_clear();
    assert(storage_take_dirty());               // an emptied ring is worth persisting
    printf("test_storage_persist: dirty flag OK\n");
}

static void test_weather_window_roundtrip_and_staleness(void) {
    float past[WEATHER_PAST_MAX], future[WEATHER_FUTURE_MAX];
    for (int i = 0; i < WEATHER_PAST_MAX; i++)   past[i] = 20.0f + i * 0.1f;
    for (int i = 0; i < WEATHER_FUTURE_MAX; i++) future[i] = 25.0f + i * 0.1f;
    const uint64_t set_at = 1787616000000ULL;
    weather_set(past, WEATHER_PAST_MAX, future, WEATHER_FUTURE_MAX, set_at);

    uint8_t blob[WEATHER_BLOB_MAX];
    size_t n = weather_serialize(blob, sizeof blob);
    assert(n == 2 + 8 + (WEATHER_PAST_MAX + WEATHER_FUTURE_MAX) * 4);

    // Same hour: adopted, values intact.
    weather_set(NULL, 0, NULL, 0, 0);
    assert(weather_restore(blob, n, set_at + 60000ULL, WEATHER_RESTORE_MAX_AGE_MS));
    const float *p = NULL, *f = NULL;
    assert(weather_get_past(&p) == WEATHER_PAST_MAX);
    assert(weather_get_future(&f) == WEATHER_FUTURE_MAX);
    assert(fabsf(p[0] - 20.0f) < 1e-6f);
    assert(fabsf(p[WEATHER_PAST_MAX - 1] - (20.0f + 47 * 0.1f)) < 1e-5f);
    assert(fabsf(f[WEATHER_FUTURE_MAX - 1] - (25.0f + 23 * 0.1f)) < 1e-5f);
    assert(weather_updated_ms() == set_at);

    // Two hours later the window describes a different hour than the one being
    // inferred: it must be refused, because nothing downstream would catch it.
    assert(!weather_restore(blob, n, set_at + 2 * HOUR_MS, WEATHER_RESTORE_MAX_AGE_MS));
    // No clock, or a stamp in the future: equally unusable.
    assert(!weather_restore(blob, n, 0, WEATHER_RESTORE_MAX_AGE_MS));
    assert(!weather_restore(blob, n, set_at - 1000, WEATHER_RESTORE_MAX_AGE_MS));
    // Truncated.
    assert(!weather_restore(blob, n - 1, set_at, WEATHER_RESTORE_MAX_AGE_MS));
    printf("test_storage_persist: TA window round-trip + staleness OK\n");
}

// Provisional stamps are uptime from the power cycle that took them. After a reboot
// the next first sync would shift them by THIS cycle's offset, so they are dropped.
static void test_restore_drops_previous_cycle_provisional(void) {
    storage_init();
    const uint64_t base = 1787616000000ULL;
    savia_reading_t real1 = mk(base, 1, 10, 0.5f);
    savia_reading_t prov1 = mk(45000, 1, 10, 0.6f);          // 45 s after an old boot
    savia_reading_t prov2 = mk(3645000, 1, 30, 0.7f);
    savia_reading_t real2 = mk(base + HOUR_MS, 1, 30, 0.8f);
    storage_append_reading(&real1);
    storage_append_reading(&prov1);
    storage_append_reading(&prov2);
    storage_append_reading(&real2);
    static uint8_t blob[SAVIA_STORAGE_BLOB_MAX];
    size_t n = storage_serialize(blob, sizeof blob);

    storage_init();                                          // "reboot"
    assert(storage_restore(blob, n) == 2);
    assert(storage_take_dirty());                            // flash copy is now stale
    assert(storage_reading_at(0)->ts_ms == base);
    assert(storage_reading_at(1)->ts_ms == base + HOUR_MS);
    assert(storage_rebase_provisional(3600000ULL) == 0);     // nothing left to shift
}

int main(void) {
    test_roundtrip();
    test_restore_drops_previous_cycle_provisional();
    test_wrapped_ring_keeps_fifo_order();
    test_malformed_blobs_are_refused_whole();
    test_dirty_tracks_every_mutation();
    test_weather_window_roundtrip_and_staleness();
    printf("test_storage_persist: OK\n");
    return 0;
}
