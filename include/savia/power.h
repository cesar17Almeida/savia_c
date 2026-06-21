// Power management: deep sleep between cycles + wake on button (GPIO) or timer.
// This is the energy win of the Pico over the Pi Zero (which has no real sleep).
#ifndef SAVIA_POWER_H
#define SAVIA_POWER_H

#include "savia/config.h"

typedef enum {
    SAVIA_WAKE_TIMER = 0,   // slept the full sleep_seconds
    SAVIA_WAKE_BUTTON = 1,   // woken early by the button/switch
} savia_wake_reason_t;

void power_init(const station_config_t *cfg);

// Enter deep sleep for `seconds` (the scheduler caps this to the next mandatory
// wake). Returns when the time elapses or the wake button fires, and tells which.
savia_wake_reason_t power_deep_sleep(const station_config_t *cfg, uint32_t seconds);

#endif // SAVIA_POWER_H
