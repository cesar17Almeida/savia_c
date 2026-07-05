// Host test for the RAM storage ring + hourly aggregation + software clock.
// Pure logic, no SDK / hardware.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "savia/storage.h"
#include "savia/clock.h"
#include "savia/types.h"

int main(void) {
    // --- clock ---
    clock_set(1700000000000ULL, 5000);
    assert(clock_is_set());
    assert(clock_now(5000) == 1700000000000ULL);
    assert(clock_now(6000) == 1700000001000ULL);   // +1 s uptime
    assert(clock_last_sync_ms() == 1700000000000ULL);
    printf("test_storage: clock OK\n");

    // --- storage ---
    storage_init();
    uint64_t H = 1700000000000ULL;
    uint64_t hour0 = H - (H % 3600000ULL);

    savia_reading_t r = { .port = 1, .depth_cm = 10, .kind = READING_SOIL_MOISTURE };
    r.ts_ms = hour0 + 1000; r.value = 0.5f; storage_append_reading(&r);
    r.ts_ms = hour0 + 2000; r.value = 0.6f; storage_append_reading(&r);
    r.ts_ms = hour0 + 3000; r.value = 0.7f; storage_append_reading(&r);
    r.ts_ms = hour0 + 3600000ULL + 500; r.value = 0.8f; storage_append_reading(&r);  // next hour

    savia_reading_t out[16];
    assert(storage_query_raw(0, UINT64_MAX, 0, out, 16) == 4);
    assert(storage_count_raw(0, UINT64_MAX) == 4);
    assert(storage_query_raw(0, hour0 + 3600000ULL, 0, out, 16) == 3);   // range filter
    assert(storage_query_raw(0, UINT64_MAX, 2, out, 16) == 2);           // limit

    savia_aggregate_t aggs[8];
    size_t na = storage_aggregate_hourly(0, UINT64_MAX, 0, aggs, 8);
    assert(na == 2);
    int b0 = -1;
    for (size_t i = 0; i < na; i++) if (aggs[i].hour_ms == hour0) b0 = (int) i;
    assert(b0 >= 0);
    assert(aggs[b0].count == 3);
    assert(aggs[b0].min == 0.5f && aggs[b0].max == 0.7f);
    assert(aggs[b0].mean > 0.59f && aggs[b0].mean < 0.61f);
    printf("test_storage: ring + aggregation OK\n");

    // --- predictions (empty -> append -> query) ---
    assert(storage_count_pred(0, UINT64_MAX) == 0);
    savia_prediction_t p;
    memset(&p, 0, sizeof(p));
    p.ts_ms = hour0; p.value = 0.66f;
    strcpy(p.model, "lstm-hs30");
    strcpy(p.kind, "hs30_forecast");
    storage_append_prediction(&p);
    savia_prediction_t po[4];
    assert(storage_query_pred(0, UINT64_MAX, 0, po, 4) == 1);
    storage_clear_predictions();                          // wipes preds, keeps readings
    assert(storage_count_pred(0, UINT64_MAX) == 0);
    assert(storage_count_raw(0, UINT64_MAX) == 4);
    printf("test_storage: predictions OK\n");

    // --- upsert: create then update by (ts_ms, port, kind, depth_cm) ---
    storage_clear();
    bool created;
    savia_reading_t u = { .ts_ms = hour0 + 1000, .port = 1, .depth_cm = 30,
                          .kind = READING_SOIL_MOISTURE, .value = 0.70f };
    assert(storage_upsert_reading(&u, &created) && created);          // new
    assert(storage_count_raw(0, UINT64_MAX) == 1);
    u.value = 0.81f;
    assert(storage_upsert_reading(&u, &created) && !created);         // same key -> update
    assert(storage_count_raw(0, UINT64_MAX) == 1);                    // no new row
    savia_reading_t uo[4];
    assert(storage_query_raw(0, UINT64_MAX, 0, uo, 4) == 1);
    assert(uo[0].value > 0.80f && uo[0].value < 0.82f);              // value overwritten
    u.depth_cm = 10;                                                  // different depth -> new row
    assert(storage_upsert_reading(&u, &created) && created);
    assert(storage_count_raw(0, UINT64_MAX) == 2);
    u.ts_ms = hour0 + 2000;                                           // different ts -> new row
    assert(storage_upsert_reading(&u, &created) && created);
    assert(storage_count_raw(0, UINT64_MAX) == 3);
    printf("test_storage: upsert OK\n");

    // --- clear wipes readings + predictions ---
    storage_clear();
    assert(storage_count_raw(0, UINT64_MAX) == 0);
    assert(storage_count_pred(0, UINT64_MAX) == 0);
    printf("test_storage: clear OK\n");

    // --- provisional back-fill: uptime-stamped rows get rebased, epoch rows don't ---
    storage_clear();
    savia_reading_t pr = { .port = 1, .depth_cm = 10, .kind = READING_SOIL_MOISTURE };
    pr.ts_ms = 45000;              pr.value = 0.5f; storage_append_reading(&pr);   // captured at uptime 45 s
    pr.ts_ms = 3645000;            pr.value = 0.6f; storage_append_reading(&pr);   // captured at uptime ~1 h
    pr.ts_ms = hour0 + 1000;       pr.value = 0.7f; storage_append_reading(&pr);   // already wall-stamped
    assert(pr.ts_ms >= SAVIA_TS_PROVISIONAL_MAX);   // sanity: epoch is above the threshold
    // Sync at uptime 100 s -> epoch H: delta = H - 100000 (what clock.c keeps as base).
    uint64_t delta = H - 100000ULL;
    assert(storage_rebase_provisional(delta) == 2);                    // only the 2 provisional rows
    savia_reading_t ro[4];
    assert(storage_query_raw(0, UINT64_MAX, 0, ro, 4) == 3);
    assert(ro[0].ts_ms == 45000 + delta);                              // uptime + base = wall capture time
    assert(ro[1].ts_ms == 3645000 + delta);
    assert(ro[2].ts_ms == hour0 + 1000);                               // untouched
    assert(storage_rebase_provisional(delta) == 0);                    // idempotent: nothing left to fix
    printf("test_storage: provisional back-fill OK\n");

    printf("test_storage: OK\n");
    return 0;
}
