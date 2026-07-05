// Time-of-day scheduler. The deep-sleep time is only the low-power granularity:
// the station MUST wake to capture on a fixed cadence and to run the daily cycle
// at a set hour, regardless of how long sleep_s is. So the actual nap is capped
// to min(sleep_s, time-until-next-scheduled-event). SDK-free, host-testable.
#ifndef SAVIA_SCHEDULER_H
#define SAVIA_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include "savia/config.h"   // savia_sensor_slot_t, SAVIA_MAX_SENSORS

typedef struct {
    // Per-sensor next-due wall-clock (ms); 0 -> capture on the first tick. Slot i
    // tracks sensor i, so each sensor keeps its own cadence.
    uint64_t next_sensor_ms[SAVIA_MAX_SENSORS];
    int32_t  last_daily_day;       // LOCAL epoch-day of the last daily fire (-1 = never)
    int32_t  last_irrigation_day;  // LOCAL epoch-day of the last irrigation mark (-1 = never)
} savia_scheduler_t;

typedef struct {
    uint8_t capture_mask;   // bit i set -> sensor i is due to be sampled now
    bool    daily;          // run the daily cycle now (inference at cfg->daily_hour LOCAL)
    bool    irrigation;     // irrigation hour reached (informative event mark)
} savia_sched_action_t;

void scheduler_init(savia_scheduler_t *s);

// Decide what is due at now_ms (wall clock, epoch ms; only call once time is set)
// and advance the schedule state. Sensor cadences are wall-clock; the daily and
// irrigation hours are LOCAL time (cfg->utc_offset_min applied internally).
savia_sched_action_t scheduler_tick(savia_scheduler_t *s, uint64_t now_ms,
                                    const station_config_t *cfg);

// Seconds to nap now: min(sleep_s, soonest sensor due, next daily hour, next
// irrigation hour). Never 0 (clamped to >= 1).
uint32_t scheduler_next_sleep_s(const savia_scheduler_t *s, uint64_t now_ms,
                                const station_config_t *cfg);

#endif // SAVIA_SCHEDULER_H
