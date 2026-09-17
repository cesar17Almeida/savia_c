#include "savia/scheduler.h"
#include "savia/sensor_catalog.h"

#define MS_PER_S    1000ULL
#define MS_PER_MIN  60000LL
#define MS_PER_HOUR 3600000ULL
#define MS_PER_DAY  86400000ULL
// A deadline further ahead than one interval plus this was set by a clock that
// has since been moved back: the sensor is due now.
#define BACKSTEP_TOLERANCE_MS 60000ULL

void scheduler_init(savia_scheduler_t *s) {
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) s->next_sensor_ms[i] = 0;  // all due on first tick
    s->last_daily_day = -1;
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

// The daily target as ms since LOCAL midnight.
static uint64_t daily_ms_of_day(const station_config_t *cfg) {
    uint32_t minutes = (uint32_t) cfg->daily_hour * 60u + (uint32_t) cfg->daily_min;
    return (uint64_t) minutes * (uint64_t) MS_PER_MIN;
}

// ms from now until the next LOCAL time-of-day boundary that still needs firing
// (today if not yet fired and not yet passed, else tomorrow). The delta is
// identical in UTC and local domains (the offset is constant).
static uint64_t ms_until_time(uint64_t local_ms, uint64_t fire_of_day, int32_t last_day) {
    int32_t today = (int32_t)(local_ms / MS_PER_DAY);
    uint64_t today_fire = (uint64_t) today * MS_PER_DAY + fire_of_day;
    if (local_ms < today_fire && last_day != today) return today_fire - local_ms;
    return (today_fire + MS_PER_DAY) - local_ms;   // already fired/passed today -> tomorrow
}

// True (and marks the day) when the LOCAL time of day has REACHED the daily target
// and it hasn't fired on this local day yet.
//
// The test is a threshold, not an equality on the hour, and that is deliberate: at
// minute resolution the target is a single minute, so an equality would be missed
// by any station that wasn't looking at that exact moment -- powered off, a deep
// sleep that overshot, or the clock jumping forward on the first LoRa sync of the
// power cycle (before that the caller doesn't tick us at all, see main.c). With
// ">=" such a station runs the cycle when it comes back instead of silently losing
// the day. The catch-up consumes the day's slot: it won't fire again until
// tomorrow. The forecast is anchored to the moment it runs, so a late run is a
// valid forecast, just issued late; and if the history isn't usable
// lstm_gather_inputs refuses it and says why.
static bool fire_at_time(uint64_t local_ms, uint64_t fire_of_day, int32_t *last_day) {
    int32_t today = (int32_t)(local_ms / MS_PER_DAY);
    uint64_t ms_of_day = local_ms % MS_PER_DAY;
    if (ms_of_day >= fire_of_day && *last_day != today) { *last_day = today; return true; }
    return false;
}

savia_sched_action_t scheduler_tick(savia_scheduler_t *s, uint64_t now_ms,
                                    const station_config_t *cfg) {
    savia_sched_action_t act = { 0, false };
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
        // A free slot has no deadline. Clearing it matters because slots are stable:
        // the sensor that later lands here would otherwise inherit the deleted one's
        // due time and stay silent for up to a full interval after being added.
        // An output slot is cleared for the same reason: it has nothing to measure,
        // so it must not hand a stale deadline to the sensor that replaces it.
        if (!savia_slot_used(&cfg->sensors[i]) ||
            sensor_type_is_output(cfg->sensors[i].type)) { s->next_sensor_ms[i] = 0; continue; }
        uint64_t interval = sensor_interval_ms(cfg->sensors, i, cfg->capture_interval_s);
        if (s->next_sensor_ms[i] == 0 || now_ms >= s->next_sensor_ms[i] ||
            s->next_sensor_ms[i] > now_ms + interval + BACKSTEP_TOLERANCE_MS) {
            act.capture_mask |= (uint8_t)(1u << i);
            uint64_t base = (s->next_sensor_ms[i] == 0 || s->next_sensor_ms[i] > now_ms)
                ? now_ms : s->next_sensor_ms[i];
            s->next_sensor_ms[i] = base + interval;
            while (s->next_sensor_ms[i] <= now_ms) s->next_sensor_ms[i] += interval;  // catch up
        }
    }

    uint64_t local = to_local_ms(now_ms, cfg->utc_offset_min);
    act.daily = fire_at_time(local, daily_ms_of_day(cfg), &s->last_daily_day);

    // Sample before inferring. The daily cycle forecasts from the last 48 h and the
    // newest step must be a real reading, not a copy (LSTM_MAX_STALE_HOURS = 0).
    // Left to their own cadence the sensors would almost never fall on the daily
    // boundary -- their due times are anchored to boot, the daily wake to the clock
    // -- so the newest bucket would belong to the previous hour and inference would
    // never run. Marking them due here costs one extra sample a day. Their own due
    // times are untouched: this is an extra reading, not a replacement.
    if (act.daily) {
        for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
            if (!savia_slot_used(&cfg->sensors[i]) ||
                sensor_type_is_output(cfg->sensors[i].type)) continue;
            act.capture_mask |= (uint8_t)(1u << i);
        }
    }
    return act;
}

void scheduler_seed_sensor(savia_scheduler_t *s, uint8_t i, uint64_t last_capture_ms,
                           const station_config_t *cfg) {
    if (i >= SAVIA_MAX_SENSORS || last_capture_ms == 0) return;
    s->next_sensor_ms[i] = last_capture_ms +
        sensor_interval_ms(cfg->sensors, i, cfg->capture_interval_s);
}

uint32_t scheduler_next_sleep_s(const savia_scheduler_t *s, uint64_t now_ms,
                                const station_config_t *cfg) {
    // Soonest sensor due. Output slots are skipped: waking to drive nothing is pure
    // battery. With no INPUT configured this stays "very far" and the daily wake
    // (always finite) bounds the nap.
    uint64_t next = (uint64_t) -1;
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
        if (!savia_slot_used(&cfg->sensors[i]) ||
            sensor_type_is_output(cfg->sensors[i].type)) continue;
        uint64_t interval = sensor_interval_ms(cfg->sensors, i, cfg->capture_interval_s);
        uint64_t until = (s->next_sensor_ms[i] > now_ms)
            ? (s->next_sensor_ms[i] - now_ms) : interval;
        if (until > interval + BACKSTEP_TOLERANCE_MS) until = 0;   // stale deadline: due now
        if (until < next) next = until;
    }

    uint64_t local = to_local_ms(now_ms, cfg->utc_offset_min);
    uint64_t until_daily = ms_until_time(local, daily_ms_of_day(cfg), s->last_daily_day);
    if (until_daily < next) next = until_daily;

    uint64_t cap = (uint64_t) cfg->sleep_seconds * MS_PER_S;
    if (cap > 0 && cap < next) next = cap;

    uint32_t secs = (uint32_t)((next + MS_PER_S - 1) / MS_PER_S);  // round up
    return secs == 0 ? 1 : secs;
}
