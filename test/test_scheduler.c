// Host test for the time-of-day scheduler (no SDK, no hardware). daily_hour and
// irrigation_hour are LOCAL time (utc_offset_min); sensor cadences are wall-clock.
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
    s.gpio2 = SAVIA_GPIO_NONE;
    s.sample_interval_s = 0;   // follow capture_interval_s
    return s;
}

// A cfg with the fields the scheduler consumes. Offsets/hours vary per test.
static station_config_t mkcfg(uint8_t nsensors, uint32_t capture_s, uint8_t daily_h,
                              int16_t offset_min, uint8_t irr_h, uint32_t sleep_s) {
    station_config_t c;
    config_load_defaults(&c);
    c.sensor_count = nsensors;
    for (uint8_t i = 0; i < nsensors; i++) c.sensors[i] = one_global((uint8_t)(2 + 4 * i));
    c.capture_interval_s = capture_s;
    c.daily_hour = daily_h;
    c.utc_offset_min = offset_min;
    c.irrigation_hour = irr_h;
    c.sleep_seconds = sleep_s;
    return c;
}

int main(void) {
    savia_scheduler_t s;
    savia_sched_action_t a;

    // --- capture cadence (hourly, one sensor on the global interval) ---
    station_config_t cfg = mkcfg(1, 3600, 20, 0, 6, 3600);
    scheduler_init(&s);
    a = scheduler_tick(&s, 0, &cfg);                 // first tick always captures
    assert(a.capture_mask == 0x1 && !a.daily);
    a = scheduler_tick(&s, 1000, &cfg);              // 1 s later, nothing due
    assert(a.capture_mask == 0 && !a.daily);
    a = scheduler_tick(&s, HOUR_MS, &cfg);           // exactly 1 h -> capture
    assert(a.capture_mask == 0x1);
    a = scheduler_tick(&s, HOUR_MS + 500, &cfg);
    assert(a.capture_mask == 0);
    printf("test_scheduler: capture cadence OK\n");

    // --- daily fires once per day at daily_hour (UTC when offset = 0) ---
    scheduler_init(&s);
    uint64_t t20 = 20 * HOUR_MS;                     // 20:00 on epoch day 0
    a = scheduler_tick(&s, t20, &cfg);
    assert(a.daily && a.capture_mask == 0x1);
    a = scheduler_tick(&s, t20 + 600000, &cfg);      // same hour, same day -> no repeat
    assert(!a.daily);
    a = scheduler_tick(&s, t20 + DAY_MS, &cfg);      // next day, hour 20 -> fires again
    assert(a.daily);
    printf("test_scheduler: daily once-per-day OK\n");

    // --- LOCAL time: offset +120 -> local 20:00 is 18:00 UTC ---
    cfg = mkcfg(1, 3600, 20, 120, 6, 3600);
    scheduler_init(&s);
    a = scheduler_tick(&s, 18 * HOUR_MS, &cfg);      // 18:00 UTC == 20:00 local
    assert(a.daily);
    a = scheduler_tick(&s, 20 * HOUR_MS, &cfg);      // 20:00 UTC == 22:00 local -> already fired
    assert(!a.daily);
    printf("test_scheduler: local-time daily (positive offset) OK\n");

    // --- TRAP: negative offset crossing midnight. offset -300 (UTC-5): local
    // 20:00 of day N is 01:00 UTC of day N+1. De-dup must use the LOCAL day. ---
    cfg = mkcfg(1, 3600, 20, -300, 6, 3600);
    scheduler_init(&s);
    uint64_t utc_0100_d1 = DAY_MS + 1 * HOUR_MS;     // 01:00 UTC day 1 == 20:00 local day 0
    a = scheduler_tick(&s, utc_0100_d1, &cfg);
    assert(a.daily);
    // 30 min later it's still local day 0, hour 20 -> must NOT refire.
    a = scheduler_tick(&s, utc_0100_d1 + 1800000, &cfg);
    assert(!a.daily);
    // Next local day's 20:00 (24 h later) fires again.
    a = scheduler_tick(&s, utc_0100_d1 + DAY_MS, &cfg);
    assert(a.daily);
    printf("test_scheduler: local-time daily (negative offset, midnight cross) OK\n");

    // --- irrigation fires once per local day at irrigation_hour ---
    cfg = mkcfg(1, 3600, 20, 0, 6, 3600);
    scheduler_init(&s);
    a = scheduler_tick(&s, 6 * HOUR_MS, &cfg);       // 06:00 -> irrigation mark
    assert(a.irrigation && !a.daily);
    a = scheduler_tick(&s, 6 * HOUR_MS + 60000, &cfg);
    assert(!a.irrigation);
    a = scheduler_tick(&s, 6 * HOUR_MS + DAY_MS, &cfg);
    assert(a.irrigation);
    printf("test_scheduler: irrigation once-per-day OK\n");

    // --- sleep is capped to the next mandatory wake (incl. irrigation) ---
    cfg = mkcfg(1, 3600, 20, 0, 6, 600);
    scheduler_init(&s);
    scheduler_tick(&s, 0, &cfg);                     // next capture +3600 s
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 600);       // sleep_s caps
    cfg.sleep_seconds = 100000;
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 3600);      // capture caps
    cfg.sleep_seconds = 43200;                       // 12 h must still wake hourly
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 3600);
    // With capture cadence 24 h, the nearest wake at t=0 is irrigation at 06:00.
    cfg = mkcfg(1, 86400, 20, 0, 6, 86400);
    scheduler_init(&s);
    scheduler_tick(&s, 0, &cfg);
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 6 * 3600);
    printf("test_scheduler: sleep capping OK (mandatory wakes honored)\n");

    // --- per-sensor cadences: sensor 0 hourly (global), sensor 1 every 5 min ---
    cfg = mkcfg(2, 3600, 20, 0, 6, 3600);
    cfg.sensors[1].sample_interval_s = 300;
    scheduler_init(&s);
    a = scheduler_tick(&s, 0, &cfg);                 // first tick: both due
    assert(a.capture_mask == 0x3);
    a = scheduler_tick(&s, 300000, &cfg);            // +5 min: only sensor 1
    assert(a.capture_mask == 0x2);
    a = scheduler_tick(&s, 600000, &cfg);            // +10 min: only sensor 1
    assert(a.capture_mask == 0x2);
    a = scheduler_tick(&s, HOUR_MS, &cfg);           // +1 h: both
    assert(a.capture_mask == 0x3);
    scheduler_init(&s);
    scheduler_tick(&s, 0, &cfg);
    cfg.sleep_seconds = 43200;
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 300);   // soonest sensor caps the nap
    printf("test_scheduler: per-sensor cadence OK (independent intervals + soonest wake)\n");

    printf("test_scheduler: OK\n");
    return 0;
}
