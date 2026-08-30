// Power management: deep sleep between cycles + wake on button (GPIO) or timer.
// This is the energy win of the Pico over the Pi Zero (which has no real sleep).
//
// Two sleeps. The light nap (power_deep_sleep) idles the core and keeps polling;
// it is what runs with deep sleep disabled and the fallback everywhere else. The
// real one (power_deep_sleep_off, RP2350 only) powers the chip off through the
// power manager -- P1.7: switched core, XIP cache and SRAM off, only the
// always-on domain alive -- and comes back as a reboot when the always-on timer
// alarm or the button fires. What the reboot needs to carry on where the plan
// stopped (wall clock, LoRa cadence, daily-cycle day) travels in the always-on
// scratch registers; main() reads it with power_deep_wake_info() first thing.
#ifndef SAVIA_POWER_H
#define SAVIA_POWER_H

#include "savia/config.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SAVIA_WAKE_TIMER = 0,   // slept the full sleep_seconds
    SAVIA_WAKE_BUTTON = 1,   // woken early by the button/switch
} savia_wake_reason_t;

// Hold the wake button this long at power-on to trigger a factory reset.
#define SAVIA_FACTORY_RESET_HOLD_MS 5000u

// Shorter naps are not worth a power cycle (the LPOSC calibration alone takes
// ~1 s and the reboot another ~1 s): they use the light nap.
#define SAVIA_DEEP_SLEEP_MIN_S 20u

void power_init(const station_config_t *cfg);

// Boot-time factory-reset gesture: true if the wake button is held low for the
// whole `hold_ms` window (released early -> false). Recovers a forgotten password.
bool power_reset_button_held(const station_config_t *cfg, uint32_t hold_ms);

// Light nap for `seconds` (the scheduler caps this to the next mandatory wake):
// the core idles and polls the button and the app's requests. Returns when the
// time elapses, the button fires or the app needs service, and tells which.
savia_wake_reason_t power_deep_sleep(const station_config_t *cfg, uint32_t seconds);

// --- real deep sleep (power-off + reboot) -------------------------------------

// Everything the wake must know to continue the plan instead of starting cold.
typedef struct {
    uint64_t now_wall_ms;           // wall clock (epoch ms) valid at `uptime_ms`
    uint64_t uptime_ms;             // ms-since-boot when now_wall_ms was read
    uint32_t nap_s;                 // seconds to sleep
    uint32_t lora_last_attempt_s;   // epoch s of the last LoRa cycle (0 = none)
    uint32_t lora_last_soil_hour_s; // newest soil hour already uplinked (0 = none)
    int32_t  last_daily_day;        // scheduler's last daily fire (-1 = never)
    uint32_t clock_uncertainty_ms;  // drift allowance already carried by the clock
} savia_deep_sleep_ctx_t;

// What a wake restores (fields valid when `resumed` is true).
typedef struct {
    bool     resumed;               // this boot continues a deep sleep
    bool     button;                // the button woke us (else the timer)
    uint64_t now_wall_ms;           // wall clock read from the always-on timer
    uint32_t slept_ms;              // how long the chip was off
    uint32_t lora_last_attempt_s;
    uint32_t lora_last_soil_hour_s;
    int32_t  last_daily_day;
    uint32_t clock_uncertainty_ms;  // allowance to give the clock (drift while off)
} savia_deep_wake_t;

// Read (and consume) the context left by a deep sleep. Call once, first thing in
// main(): `resumed` is false on a cold boot, after a reset, or when the timer did
// not survive. Never blocks and never logs (nothing is up yet).
void power_deep_wake_info(savia_deep_wake_t *out);

// True while the power manager can be used for the real deep sleep this power
// cycle (RP2350 and no refused request so far). RP2040: always false.
bool power_deep_sleep_available(void);

// Power the chip off until the alarm at now + nap_s or the button (active low).
// The radio must already be down (ble_radio_suspend). On success it never
// returns: the board reboots and main() finds the context. Returns false if the
// power manager refused -> the caller takes the light nap instead.
bool power_deep_sleep_off(const station_config_t *cfg, const savia_deep_sleep_ctx_t *ctx);

#endif // SAVIA_POWER_H
