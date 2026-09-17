// Milliseconds since boot, 64-bit. to_ms_since_boot() truncates to 32 bits and
// wraps after 49.7 days, which a station that never deep-sleeps reaches.
#ifndef SAVIA_UPTIME_H
#define SAVIA_UPTIME_H

#include <stdint.h>
#include "hardware/timer.h"

static inline uint64_t savia_uptime_ms(void) { return time_us_64() / 1000u; }

#endif // SAVIA_UPTIME_H
