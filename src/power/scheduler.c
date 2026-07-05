#include "savia/scheduler.h"

#define MS_PER_S    1000ULL
#define MS_PER_MIN  60000LL
#define MS_PER_HOUR 3600000ULL
#define MS_PER_DAY  86400000ULL

void scheduler_init(savia_scheduler_t *s) {
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) s->next_sensor_ms[i] = 0;  // all due on first tick
    s->last_daily_day = -1;
    s->last_irrigation_day = -1;
}

// Wall (UTC) epoch ms -> local epoch ms. Offset is minutes, may be negative; the
// epoch base (>1e12 once synced) keeps this positive for any legal offset.
static uint64_t to_local_ms(uint64_t now_ms, int16_t utc_offset_min) {
    return (uint64_t)((int64_t) now_ms + (int64_t) utc_offset_min * MS_PER_MIN);
}

// Effective cadence (ms) for sensor i: its own sample_interval_s, or the global
// default when 0; never 0 (falls back to hourly).
static uint64_t sensor_interval_ms(const savia_sensor_slot_t *sensors, uint8_t i,
                                   uint32_t default_interval_s) {
    uint32_t secs = sensors[i].sample_interval_s ? sensors[i].sample_interval_s : default_interval_s;
    uint64_t ms = (uint64_t) secs * MS_PER_S;
    return ms == 0 ? MS_PER_HOUR : ms;
}

// ms from now until the next LOCAL time-of-day `hour` boundary that still needs
// firing (today if not yet fired and not yet passed, else tomorrow). The delta is
// identical in UTC and local domains (the offset is constant).
static uint64_t ms_until_hour(uint64_t local_ms, uint8_t hour, int32_t last_day) {
    int32_t today = (int32_t)(local_ms / MS_PER_DAY);
    uint64_t today_fire = (uint64_t) today * MS_PER_DAY + (uint64_t) hour * MS_PER_HOUR;
    if (local_ms < today_fire && last_day != today) return today_fire - local_ms;
    return (today_fire + MS_PER_DAY) - local_ms;   // already fired/passed today -> tomorrow
}

// True (and marks the day) when LOCAL time-of-day `hour` is due and hasn't fired
// on this local day yet.
static bool fire_at_hour(uint64_t local_ms, uint8_t hour, int32_t *last_day) {
    int32_t today = (int32_t)(local_ms / MS_PER_DAY);
    uint32_t h = (uint32_t)((local_ms / MS_PER_HOUR) % 24);
    if (h == hour && *last_day != today) { *last_day = today; return true; }
    return false;
}

savia_sched_action_t scheduler_tick(savia_scheduler_t *s, uint64_t now_ms,
                                    const station_config_t *cfg) {
    savia_sched_action_t act = { 0, false, false };
    uint8_t count = cfg->sensor_count;
    if (count > SAVIA_MAX_SENSORS) count = SAVIA_MAX_SENSORS;

    for (uint8_t i = 0; i < count; i++) {
        uint64_t interval = sensor_interval_ms(cfg->sensors, i, cfg->capture_interval_s);
        if (s->next_sensor_ms[i] == 0 || now_ms >= s->next_sensor_ms[i]) {
            act.capture_mask |= (uint8_t)(1u << i);
            uint64_t base = (s->next_sensor_ms[i] == 0) ? now_ms : s->next_sensor_ms[i];
            s->next_sensor_ms[i] = base + interval;
            while (s->next_sensor_ms[i] <= now_ms) s->next_sensor_ms[i] += interval;  // catch up
        }
    }

    uint64_t local = to_local_ms(now_ms, cfg->utc_offset_min);
    act.daily      = fire_at_hour(local, cfg->daily_hour, &s->last_daily_day);
    act.irrigation = fire_at_hour(local, cfg->irrigation_hour, &s->last_irrigation_day);
    return act;
}

uint32_t scheduler_next_sleep_s(const savia_scheduler_t *s, uint64_t now_ms,
                                const station_config_t *cfg) {
    uint8_t count = cfg->sensor_count;
    if (count > SAVIA_MAX_SENSORS) count = SAVIA_MAX_SENSORS;

    // Soonest sensor due. With no sensors configured this stays "very far" and the
    // daily/irrigation wakes (always finite) bound the nap.
    uint64_t next = (uint64_t) -1;
    for (uint8_t i = 0; i < count; i++) {
        uint64_t interval = sensor_interval_ms(cfg->sensors, i, cfg->capture_interval_s);
        uint64_t until = (s->next_sensor_ms[i] > now_ms)
            ? (s->next_sensor_ms[i] - now_ms) : interval;
        if (until < next) next = until;
    }

    uint64_t local = to_local_ms(now_ms, cfg->utc_offset_min);
    uint64_t until_daily = ms_until_hour(local, cfg->daily_hour, s->last_daily_day);
    if (until_daily < next) next = until_daily;
    uint64_t until_irr = ms_until_hour(local, cfg->irrigation_hour, s->last_irrigation_day);
    if (until_irr < next) next = until_irr;

    uint64_t cap = (uint64_t) cfg->sleep_seconds * MS_PER_S;
    if (cap > 0 && cap < next) next = cap;

    uint32_t secs = (uint32_t)((next + MS_PER_S - 1) / MS_PER_S);  // round up
    return secs == 0 ? 1 : secs;
}
