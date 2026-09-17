#include "savia/wdt.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

static bool s_armed;
static bool s_caused_reboot;

void savia_wdt_start(void) {
    // Read before arming: watchdog_enable rewrites the scratch word this checks.
    s_caused_reboot = watchdog_enable_caused_reboot();
    watchdog_enable(SAVIA_WDT_TIMEOUT_MS, /*pause_on_debug=*/true);
    s_armed = true;
}

void savia_wdt_feed(void) {
    if (s_armed) watchdog_update();
}

void savia_wdt_sleep_ms(uint32_t ms) {
    const uint32_t step_ms = 1000;
    while (ms > 0) {
        uint32_t d = ms < step_ms ? ms : step_ms;
        savia_wdt_feed();
        sleep_ms(d);
        ms -= d;
    }
    savia_wdt_feed();
}

bool savia_wdt_caused_reboot(void) { return s_caused_reboot; }
