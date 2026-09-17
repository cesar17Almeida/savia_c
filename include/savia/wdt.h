// Hardware watchdog: resets the board when the supervisor stops feeding it (a
// wedged driver, a lost interrupt). Every wait that can exceed a few seconds
// feeds it; the main loop feeds it once per iteration.
#ifndef SAVIA_WDT_H
#define SAVIA_WDT_H

#include <stdbool.h>
#include <stdint.h>

// Longest the firmware may go without a feed (RP2040 caps the timer at ~8.3 s).
#define SAVIA_WDT_TIMEOUT_MS 8000u

// Arm the watchdog. Remembers first whether the previous reset was its doing.
void savia_wdt_start(void);
void savia_wdt_feed(void);
// sleep_ms() that keeps feeding, for waits longer than the timeout.
void savia_wdt_sleep_ms(uint32_t ms);
// True when this boot follows a reset by the watchdog armed above.
bool savia_wdt_caused_reboot(void);

#endif // SAVIA_WDT_H
