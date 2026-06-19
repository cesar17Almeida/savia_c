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

// Enter deep sleep (DORMANT). Returns when either cfg->sleep_seconds elapse or
// the wake button fires. Returns which one woke us.
savia_wake_reason_t power_deep_sleep(const station_config_t *cfg);

#endif // SAVIA_POWER_H
