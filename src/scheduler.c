#include "savia/scheduler.h"

#define MS_PER_S    1000ULL
#define MS_PER_HOUR 3600000ULL
#define MS_PER_DAY  86400000ULL

void scheduler_init(savia_scheduler_t *s) {
    s->next_capture_ms = 0;     // first tick captures immediately
    s->last_daily_day = -1;
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
                                    uint32_t capture_interval_s, uint8_t daily_hour) {
    savia_sched_action_t act = { false, false };
    uint64_t interval = (uint64_t) capture_interval_s * MS_PER_S;
    if (interval == 0) interval = MS_PER_HOUR;

    if (s->next_capture_ms == 0 || now_ms >= s->next_capture_ms) {
        act.capture = true;
        uint64_t base = (s->next_capture_ms == 0) ? now_ms : s->next_capture_ms;
        s->next_capture_ms = base + interval;
        while (s->next_capture_ms <= now_ms) s->next_capture_ms += interval;  // catch up
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
                                uint32_t sleep_s, uint32_t capture_interval_s,
                                uint8_t daily_hour) {
    uint64_t interval = (uint64_t) capture_interval_s * MS_PER_S;
    if (interval == 0) interval = MS_PER_HOUR;

    uint64_t until_capture = (s->next_capture_ms > now_ms)
        ? (s->next_capture_ms - now_ms) : interval;
    uint64_t until_daily = ms_until_daily(now_ms, daily_hour, s->last_daily_day);

    uint64_t next = until_capture < until_daily ? until_capture : until_daily;
    uint64_t cap = (uint64_t) sleep_s * MS_PER_S;
    if (cap > 0 && cap < next) next = cap;

    uint32_t secs = (uint32_t)((next + MS_PER_S - 1) / MS_PER_S);  // round up
    return secs == 0 ? 1 : secs;
}
