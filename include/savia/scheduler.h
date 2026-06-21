// Time-of-day scheduler. The deep-sleep time is only the low-power granularity:
// the station MUST wake to capture on a fixed cadence and to run the daily cycle
// at a set hour, regardless of how long sleep_s is. So the actual nap is capped
// to min(sleep_s, time-until-next-scheduled-event). SDK-free, host-testable.
#ifndef SAVIA_SCHEDULER_H
#define SAVIA_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint64_t next_capture_ms;   // 0 -> capture on the first tick
    int32_t  last_daily_day;    // epoch-day index of the last daily fire (-1 = never)
} savia_scheduler_t;

typedef struct {
    bool capture;   // sample the sensors now
    bool daily;     // run the daily cycle now (inference on RP2350; service window)
} savia_sched_action_t;

void scheduler_init(savia_scheduler_t *s);

// Decide what is due at now_ms and advance the schedule state. now_ms is the
// wall clock (epoch ms); only call once a valid time is set.
savia_sched_action_t scheduler_tick(savia_scheduler_t *s, uint64_t now_ms,
                                    uint32_t capture_interval_s, uint8_t daily_hour);

// Seconds to nap now: min(sleep_s, time until next capture, time until next
// daily hour). Never 0 (clamped to >= 1).
uint32_t scheduler_next_sleep_s(const savia_scheduler_t *s, uint64_t now_ms,
                                uint32_t sleep_s, uint32_t capture_interval_s,
                                uint8_t daily_hour);

#endif // SAVIA_SCHEDULER_H
