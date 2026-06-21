// Power management: deep sleep between cycles + wake on button (GPIO) or timer.
// This is the energy win of the Pico over the Pi Zero (which has no real sleep).
#ifndef SAVIA_POWER_H
#define SAVIA_POWER_H

#include "savia/config.h"

typedef enum {
    SAVIA_WAKE_TIMER = 0,   // slept the full sleep_seconds
    SAVIA_WAKE_BUTTON = 1,   // woken early by the button/switch
} savia_wake_reason_t;

// Hold the wake button this long at power-on to trigger a factory reset.
#define SAVIA_FACTORY_RESET_HOLD_MS 5000u

void power_init(const station_config_t *cfg);

// Boot-time factory-reset gesture: true if the wake button is held low for the
// whole `hold_ms` window (released early -> false). Recovers a forgotten password.
bool power_reset_button_held(const station_config_t *cfg, uint32_t hold_ms);

// Enter deep sleep for `seconds` (the scheduler caps this to the next mandatory
// wake). Returns when the time elapses or the wake button fires, and tells which.
savia_wake_reason_t power_deep_sleep(const station_config_t *cfg, uint32_t seconds);

#endif // SAVIA_POWER_H
