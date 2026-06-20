#include "savia/clock.h"

static uint64_t s_epoch_base_ms;   // epoch_ms - uptime_ms at last sync
static uint64_t s_last_sync_ms;
static bool     s_set;

void clock_set(uint64_t epoch_ms, uint64_t uptime_ms) {
    s_epoch_base_ms = epoch_ms - uptime_ms;
    s_last_sync_ms = epoch_ms;
    s_set = true;
}

uint64_t clock_now(uint64_t uptime_ms) {
    if (!s_set) return 0;
    return s_epoch_base_ms + uptime_ms;
}

bool clock_is_set(void) { return s_set; }

uint64_t clock_last_sync_ms(void) { return s_last_sync_ms; }
