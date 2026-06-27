// Host test for the time-of-day scheduler (no SDK, no hardware).
#include <assert.h>
#include <stdio.h>
#include "savia/scheduler.h"
#include "savia/config.h"

#define HOUR_MS 3600000ULL
#define DAY_MS  86400000ULL

// One sensor whose cadence follows the global default (sample_interval_s == 0).
static savia_sensor_slot_t one_global(uint8_t gpio) {
    savia_sensor_slot_t s = { 0 };
    s.type = SENSOR_SDI12_AQUACHECK;
    s.gpio = gpio;
    s.sample_interval_s = 0;   // follow default_interval_s
    return s;
}

int main(void) {
    savia_scheduler_t s;
    savia_sched_action_t a;
    savia_sensor_slot_t sensors[2];

    // --- capture cadence (hourly, one sensor on the global interval) ---
    sensors[0] = one_global(2);
    scheduler_init(&s);
    a = scheduler_tick(&s, 0, sensors, 1, 3600, 20);         // first tick always captures
    assert(a.capture_mask == 0x1 && !a.daily);
    a = scheduler_tick(&s, 1000, sensors, 1, 3600, 20);      // 1 s later, nothing due
    assert(a.capture_mask == 0 && !a.daily);
    a = scheduler_tick(&s, HOUR_MS, sensors, 1, 3600, 20);   // exactly 1 h -> capture
    assert(a.capture_mask == 0x1);
    a = scheduler_tick(&s, HOUR_MS + 500, sensors, 1, 3600, 20);
    assert(a.capture_mask == 0);
    printf("test_scheduler: capture cadence OK\n");

    // --- daily fires once per day at daily_hour ---
    scheduler_init(&s);
    uint64_t t20 = 20 * HOUR_MS;                             // 20:00 on epoch day 0
    a = scheduler_tick(&s, t20, sensors, 1, 3600, 20);
    assert(a.daily && a.capture_mask == 0x1);
    a = scheduler_tick(&s, t20 + 600000, sensors, 1, 3600, 20); // same hour, same day -> no repeat
    assert(!a.daily);
    a = scheduler_tick(&s, t20 + DAY_MS, sensors, 1, 3600, 20);  // next day, hour 20 -> fires again
    assert(a.daily);
    printf("test_scheduler: daily once-per-day OK\n");

    // --- sleep is capped to the next mandatory wake ---
    scheduler_init(&s);
    scheduler_tick(&s, 0, sensors, 1, 3600, 20);            // next capture at +3600 s; hour 0
    // capture is 3600 s away, daily (20:00) is 72000 s away.
    assert(scheduler_next_sleep_s(&s, 0, 600, sensors, 1, 3600, 20) == 600);     // sleep_s caps
    assert(scheduler_next_sleep_s(&s, 0, 100000, sensors, 1, 3600, 20) == 3600); // capture caps
    // KEY: even a 12 h sleep_s must wake at the hourly capture.
    assert(scheduler_next_sleep_s(&s, 0, 43200, sensors, 1, 3600, 20) == 3600);
    printf("test_scheduler: sleep capping OK (mandatory wake honored)\n");

    // --- per-sensor cadences: sensor 0 hourly (global), sensor 1 every 5 min ---
    sensors[0] = one_global(2);                             // 3600 s via default
    sensors[1] = one_global(6);
    sensors[1].sample_interval_s = 300;                     // own cadence: 5 min
    scheduler_init(&s);
    a = scheduler_tick(&s, 0, sensors, 2, 3600, 20);        // first tick: both due
    assert(a.capture_mask == 0x3);
    a = scheduler_tick(&s, 300000, sensors, 2, 3600, 20);   // +5 min: only sensor 1
    assert(a.capture_mask == 0x2);
    a = scheduler_tick(&s, 600000, sensors, 2, 3600, 20);   // +10 min: only sensor 1
    assert(a.capture_mask == 0x2);
    a = scheduler_tick(&s, HOUR_MS, sensors, 2, 3600, 20);  // +1 h: sensor 0 (hourly) + sensor 1
    assert(a.capture_mask == 0x3);
    // The nap wakes at the SOONEST sensor: just after the first tick sensor 1 is due
    // in 5 min, so even a 12 h sleep_s caps to 300 s.
    scheduler_init(&s);
    scheduler_tick(&s, 0, sensors, 2, 3600, 20);
    assert(scheduler_next_sleep_s(&s, 0, 43200, sensors, 2, 3600, 20) == 300);
    printf("test_scheduler: per-sensor cadence OK (independent intervals + soonest wake)\n");

    printf("test_scheduler: OK\n");
    return 0;
}
