// Host test for the time-of-day scheduler (no SDK, no hardware).
#include <assert.h>
#include <stdio.h>
#include "savia/scheduler.h"

#define HOUR_MS 3600000ULL
#define DAY_MS  86400000ULL

int main(void) {
    savia_scheduler_t s;

    // --- capture cadence (hourly) ---
    scheduler_init(&s);
    savia_sched_action_t a;
    a = scheduler_tick(&s, 0, 3600, 20);          // first tick always captures
    assert(a.capture && !a.daily);
    a = scheduler_tick(&s, 1000, 3600, 20);       // 1 s later, nothing due
    assert(!a.capture && !a.daily);
    a = scheduler_tick(&s, HOUR_MS, 3600, 20);    // exactly 1 h -> capture
    assert(a.capture);
    a = scheduler_tick(&s, HOUR_MS + 500, 3600, 20);
    assert(!a.capture);
    printf("test_scheduler: capture cadence OK\n");

    // --- daily fires once per day at daily_hour ---
    scheduler_init(&s);
    uint64_t t20 = 20 * HOUR_MS;                   // 20:00 on epoch day 0
    a = scheduler_tick(&s, t20, 3600, 20);
    assert(a.daily && a.capture);
    a = scheduler_tick(&s, t20 + 600000, 3600, 20); // same hour, same day -> no repeat
    assert(!a.daily);
    a = scheduler_tick(&s, t20 + DAY_MS, 3600, 20); // next day, hour 20 -> fires again
    assert(a.daily);
    printf("test_scheduler: daily once-per-day OK\n");

    // --- sleep is capped to the next mandatory wake ---
    scheduler_init(&s);
    scheduler_tick(&s, 0, 3600, 20);               // next capture at +3600 s; hour 0
    // capture is 3600 s away, daily (20:00) is 72000 s away.
    assert(scheduler_next_sleep_s(&s, 0, 600, 3600, 20) == 600);     // sleep_s caps
    assert(scheduler_next_sleep_s(&s, 0, 100000, 3600, 20) == 3600); // capture caps
    // KEY: even a 12 h sleep_s must wake at the hourly capture.
    assert(scheduler_next_sleep_s(&s, 0, 43200, 3600, 20) == 3600);
    printf("test_scheduler: sleep capping OK (mandatory wake honored)\n");

    printf("test_scheduler: OK\n");
    return 0;
}
