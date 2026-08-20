// Host test for the time-of-day scheduler (no SDK, no hardware). The daily cycle
// fires at daily_hour:daily_min in LOCAL time (utc_offset_min); sensor cadences are
// wall-clock. mkcfg() leaves daily_min at its default 0; the minute-resolution
// tests set it explicitly.
#include <assert.h>
#include <stdio.h>
#include "savia/scheduler.h"
#include "savia/config.h"

#define MIN_MS  60000ULL
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
                              int16_t offset_min, uint32_t sleep_s) {
    station_config_t c;
    config_load_defaults(&c);
    for (uint8_t i = 0; i < nsensors; i++) c.sensors[i] = one_global((uint8_t)(2 + 4 * i));
    c.capture_interval_s = capture_s;
    c.daily_hour = daily_h;
    c.utc_offset_min = offset_min;
    c.sleep_seconds = sleep_s;
    return c;
}

int main(void) {
    savia_scheduler_t s;
    savia_sched_action_t a;

    // --- capture cadence (hourly, one sensor on the global interval) ---
    station_config_t cfg = mkcfg(1, 3600, 20, 0, 3600);
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
    cfg = mkcfg(1, 3600, 20, 120, 3600);
    scheduler_init(&s);
    a = scheduler_tick(&s, 18 * HOUR_MS, &cfg);      // 18:00 UTC == 20:00 local
    assert(a.daily);
    a = scheduler_tick(&s, 20 * HOUR_MS, &cfg);      // 20:00 UTC == 22:00 local -> already fired
    assert(!a.daily);
    printf("test_scheduler: local-time daily (positive offset) OK\n");

    // --- TRAP: negative offset crossing midnight. offset -300 (UTC-5): local
    // 20:00 of day N is 01:00 UTC of day N+1. De-dup must use the LOCAL day. ---
    cfg = mkcfg(1, 3600, 20, -300, 3600);
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

    // --- minute resolution: 20:30 must not fire at 20:00 ---
    cfg = mkcfg(1, 3600, 20, 0, 3600);
    cfg.daily_min = 30;
    scheduler_init(&s);
    a = scheduler_tick(&s, 20 * HOUR_MS, &cfg);              // 20:00 -> too early
    assert(!a.daily);
    a = scheduler_tick(&s, 20 * HOUR_MS + 29 * MIN_MS, &cfg); // 20:29 -> still early
    assert(!a.daily);
    a = scheduler_tick(&s, 20 * HOUR_MS + 30 * MIN_MS, &cfg); // 20:30 -> fires
    assert(a.daily);
    a = scheduler_tick(&s, 20 * HOUR_MS + 45 * MIN_MS, &cfg); // same day -> no repeat
    assert(!a.daily);
    a = scheduler_tick(&s, DAY_MS + 20 * HOUR_MS + 30 * MIN_MS, &cfg);  // next day
    assert(a.daily);
    printf("test_scheduler: minute resolution OK\n");

    // --- the nap lands on the minute, not the hour ---
    cfg = mkcfg(1, 86400, 20, 0, 86400);
    cfg.daily_min = 30;
    scheduler_init(&s);
    scheduler_tick(&s, 0, &cfg);
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 20 * 3600 + 30 * 60);
    printf("test_scheduler: sleep capping honors daily_min OK\n");

    // --- catch-up: a station whose first tick of the day is already past the
    // target runs the cycle instead of losing the day (powered off, sleep
    // overshoot, or the clock jumping forward on the first LoRa sync) ---
    cfg = mkcfg(1, 3600, 20, 0, 3600);
    cfg.daily_min = 30;
    scheduler_init(&s);
    a = scheduler_tick(&s, 23 * HOUR_MS, &cfg);              // back at 23:00, target was 20:30
    assert(a.daily);
    // The catch-up consumed the day's slot: no second fire on the same local day.
    a = scheduler_tick(&s, 23 * HOUR_MS + 59 * MIN_MS, &cfg);
    assert(!a.daily);
    // Tomorrow is back on schedule, at the configured time and not before.
    a = scheduler_tick(&s, DAY_MS + 20 * HOUR_MS, &cfg);
    assert(!a.daily);
    a = scheduler_tick(&s, DAY_MS + 20 * HOUR_MS + 30 * MIN_MS, &cfg);
    assert(a.daily);
    printf("test_scheduler: late-wake catch-up OK\n");

    // --- catch-up must not reach back across midnight: waking at 00:10 with the
    // target at 20:30 belongs to a NEW local day whose target is still ahead ---
    cfg = mkcfg(1, 3600, 20, 0, 3600);
    cfg.daily_min = 30;
    scheduler_init(&s);
    a = scheduler_tick(&s, DAY_MS + 10 * MIN_MS, &cfg);      // 00:10 of day 1
    assert(!a.daily);
    printf("test_scheduler: catch-up does not cross midnight OK\n");

    // --- sleep is capped to the next mandatory wake ---
    cfg = mkcfg(1, 3600, 20, 0, 600);
    scheduler_init(&s);
    scheduler_tick(&s, 0, &cfg);                     // next capture +3600 s
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 600);       // sleep_s caps
    cfg.sleep_seconds = 100000;
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 3600);      // capture caps
    cfg.sleep_seconds = 43200;                       // 12 h must still wake hourly
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 3600);
    // With capture cadence 24 h, the nearest wake at t=0 is the daily at 20:00.
    cfg = mkcfg(1, 86400, 20, 0, 86400);
    scheduler_init(&s);
    scheduler_tick(&s, 0, &cfg);
    assert(scheduler_next_sleep_s(&s, 0, &cfg) == 20 * 3600);
    printf("test_scheduler: sleep capping OK (mandatory wakes honored)\n");

    // --- per-sensor cadences: sensor 0 hourly (global), sensor 1 every 5 min ---
    cfg = mkcfg(2, 3600, 20, 0, 3600);
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

    // --- output slots never schedule a capture nor bound the sleep ---
    // An actuator occupies a slot like any sensor, but it produces no reading, so
    // waking for it would be pure battery burnt on nothing.
    {
        station_config_t c = mkcfg(1, 3600, 20, 0, 86400);
        c.sensors[1].type = SENSOR_ACTUATOR_DIGITAL;      // port 2 = digital output
        c.sensors[1].gpio = 15;
        c.sensors[1].gpio2 = SAVIA_GPIO_NONE;
        c.sensors[1].sample_interval_s = 60;              // ignored: nothing to measure
        savia_scheduler_t so;
        scheduler_init(&so);
        a = scheduler_tick(&so, 0, &c);
        assert(a.capture_mask == 0x1);                    // only the sensor, never the output
        a = scheduler_tick(&so, 60000, &c);               // +1 min: the output's "cadence"
        assert(a.capture_mask == 0);                      // must not fire
        // The nap is bounded by the sensor (1 h), not by the output's 60 s -- which
        // would otherwise wake the board 60x per hour to drive nothing.
        assert(scheduler_next_sleep_s(&so, 0, &c) == 3600);

        // With ONLY an output configured, nothing captures and the daily wake is
        // the sole bound.
        c.sensors[0].type = SENSOR_NONE;
        scheduler_init(&so);
        a = scheduler_tick(&so, 0, &c);
        assert(a.capture_mask == 0);
        assert(scheduler_next_sleep_s(&so, 0, &c) == 20 * 3600);   // daily at 20:00
        printf("test_scheduler: output slots never capture nor cap the nap OK\n");
    }

    // --- the daily tick samples first ---
    // The model's newest step must be a real reading of the hour being inferred
    // (LSTM_MAX_STALE_HOURS = 0). Sensor due times are anchored to boot and the
    // daily wake to the clock, so they would rarely coincide on their own.
    {
        station_config_t c = mkcfg(2, 3600, 20, 0, 3600);
        c.sensors[1].type = SENSOR_ACTUATOR_DIGITAL;   // an output must NOT be sampled
        c.sensors[1].gpio = 15;
        savia_scheduler_t sd;
        scheduler_init(&sd);
        scheduler_tick(&sd, 0, &c);                   // first tick: sensor 0 due, next in 1 h
        a = scheduler_tick(&sd, 60000, &c);           // +1 min: nothing due
        assert(a.capture_mask == 0 && !a.daily);
        a = scheduler_tick(&sd, 20 * HOUR_MS, &c);    // 20:00 -> daily forces a sample
        assert(a.daily && a.capture_mask == 0x1);     // the sensor, never the actuator

        // The forced sample does not consume the sensor's own slot: it stays on its
        // cadence rather than being pushed an hour out by the daily tick.
        a = scheduler_tick(&sd, 21 * HOUR_MS, &c);
        assert(!a.daily && a.capture_mask == 0x1);
        printf("test_scheduler: daily tick samples first OK\n");
    }

    printf("test_scheduler: OK\n");
    // --- stable slots: a freed slot must not hand its due time to the next sensor ---
    // Slots never shift, so slot 1 can sit empty for a while and then be filled by a
    // different sensor. If its old deadline survived, the new one would stay silent
    // for up to a full interval after being added.
    {
        station_config_t c = mkcfg(2, 3600, 20, 0, 3600);
        savia_scheduler_t sl;
        scheduler_init(&sl);
        uint64_t t = 1780000000000ULL;
        a = scheduler_tick(&sl, t, &c);
        assert((a.capture_mask & 0x3) == 0x3);            // both captured, next in 1 h
        a = scheduler_tick(&sl, t + 60000, &c);
        assert(a.capture_mask == 0);                      // neither due yet

        // Delete the sensor on port 2: its slot goes free and its deadline dies with it.
        c.sensors[1].type = SENSOR_NONE;
        a = scheduler_tick(&sl, t + 120000, &c);
        assert(a.capture_mask == 0);                      // nothing to capture there

        // Add a new sensor on that same port: it captures on the very next tick.
        c.sensors[1] = one_global(20);
        a = scheduler_tick(&sl, t + 180000, &c);
        assert((a.capture_mask & (1u << 1)) != 0);        // the newcomer fires now
        assert((a.capture_mask & (1u << 0)) == 0);        // the untouched one still waits
        printf("test_scheduler: freed slot drops its deadline OK\n");
    }

    return 0;
}
