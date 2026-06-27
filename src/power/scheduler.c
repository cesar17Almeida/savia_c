#include "savia/scheduler.h"

#define MS_PER_S    1000ULL
#define MS_PER_HOUR 3600000ULL
#define MS_PER_DAY  86400000ULL

void scheduler_init(savia_scheduler_t *s) {
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) s->next_sensor_ms[i] = 0;  // all due on first tick
    s->last_daily_day = -1;
}

// Effective cadence (ms) for sensor i: its own sample_interval_s, or the global
// default when 0; never 0 (falls back to hourly).
static uint64_t sensor_interval_ms(const savia_sensor_slot_t *sensors, uint8_t i,
                                   uint32_t default_interval_s) {
    uint32_t secs = sensors[i].sample_interval_s ? sensors[i].sample_interval_s : default_interval_s;
    uint64_t ms = (uint64_t) secs * MS_PER_S;
    return ms == 0 ? MS_PER_HOUR : ms;
}

// ms from now until the next time-of-day `daily_hour` boundary that still needs
// firing (today if not yet fired and not yet passed, else tomorrow).
static uint64_t ms_until_daily(uint64_t now_ms, uint8_t daily_hour, int32_t last_daily_day) {
    int32_t today = (int32_t)(now_ms / MS_PER_DAY);
    uint64_t today_fire = (uint64_t) today * MS_PER_DAY + (uint64_t) daily_hour * MS_PER_HOUR;
    if (now_ms < today_fire && last_daily_day != today) return today_fire - now_ms;
    return (today_fire + MS_PER_DAY) - now_ms;   // already fired/passed today -> tomorrow
}

savia_sched_action_t scheduler_tick(savia_scheduler_t *s, uint64_t now_ms,
                                    const savia_sensor_slot_t *sensors, uint8_t count,
                                    uint32_t default_interval_s, uint8_t daily_hour) {
    savia_sched_action_t act = { 0, false };
    if (count > SAVIA_MAX_SENSORS) count = SAVIA_MAX_SENSORS;

    for (uint8_t i = 0; i < count; i++) {
        uint64_t interval = sensor_interval_ms(sensors, i, default_interval_s);
        if (s->next_sensor_ms[i] == 0 || now_ms >= s->next_sensor_ms[i]) {
            act.capture_mask |= (uint8_t)(1u << i);
            uint64_t base = (s->next_sensor_ms[i] == 0) ? now_ms : s->next_sensor_ms[i];
            s->next_sensor_ms[i] = base + interval;
            while (s->next_sensor_ms[i] <= now_ms) s->next_sensor_ms[i] += interval;  // catch up
        }
    }

    int32_t today = (int32_t)(now_ms / MS_PER_DAY);
    uint32_t hour = (uint32_t)((now_ms / MS_PER_HOUR) % 24);
    if (hour == daily_hour && s->last_daily_day != today) {
        act.daily = true;
        s->last_daily_day = today;
    }
    return act;
}

uint32_t scheduler_next_sleep_s(const savia_scheduler_t *s, uint64_t now_ms,
                                uint32_t sleep_s,
                                const savia_sensor_slot_t *sensors, uint8_t count,
                                uint32_t default_interval_s, uint8_t daily_hour) {
    if (count > SAVIA_MAX_SENSORS) count = SAVIA_MAX_SENSORS;

    // Soonest sensor due. With no sensors configured this stays "very far" and the
    // daily wake (always finite) bounds the nap.
    uint64_t next = (uint64_t) -1;
    for (uint8_t i = 0; i < count; i++) {
        uint64_t interval = sensor_interval_ms(sensors, i, default_interval_s);
        uint64_t until = (s->next_sensor_ms[i] > now_ms)
            ? (s->next_sensor_ms[i] - now_ms) : interval;
        if (until < next) next = until;
    }

    uint64_t until_daily = ms_until_daily(now_ms, daily_hour, s->last_daily_day);
    if (until_daily < next) next = until_daily;

    uint64_t cap = (uint64_t) sleep_s * MS_PER_S;
    if (cap > 0 && cap < next) next = cap;

    uint32_t secs = (uint32_t)((next + MS_PER_S - 1) / MS_PER_S);  // round up
    return secs == 0 ? 1 : secs;
}
