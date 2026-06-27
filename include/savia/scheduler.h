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
    int32_t  last_daily_day;    // epoch-day index of the last daily fire (-1 = never)
} savia_scheduler_t;

typedef struct {
    uint8_t capture_mask;   // bit i set -> sensor i is due to be sampled now
    bool    daily;          // run the daily cycle now (inference on RP2350; service window)
} savia_sched_action_t;

void scheduler_init(savia_scheduler_t *s);

// Decide which sensors are due at now_ms and advance the schedule state. Each
// sensor fires on its own cadence: sensors[i].sample_interval_s, or
// default_interval_s when that is 0. now_ms is the wall clock (epoch ms); only
// call once a valid time is set.
savia_sched_action_t scheduler_tick(savia_scheduler_t *s, uint64_t now_ms,
                                    const savia_sensor_slot_t *sensors, uint8_t count,
                                    uint32_t default_interval_s, uint8_t daily_hour);

// Seconds to nap now: min(sleep_s, time until the soonest sensor is due, time
// until the next daily hour). Never 0 (clamped to >= 1).
uint32_t scheduler_next_sleep_s(const savia_scheduler_t *s, uint64_t now_ms,
                                uint32_t sleep_s,
                                const savia_sensor_slot_t *sensors, uint8_t count,
                                uint32_t default_interval_s, uint8_t daily_hour);

#endif // SAVIA_SCHEDULER_H
