// Station configuration. Defaults are compiled in; the app can override fields
// over BLE at runtime (persisted to flash) -- e.g. sleep time and sensor pins.
#ifndef SAVIA_CONFIG_H
#define SAVIA_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "savia/auth.h"

#define SAVIA_MAX_SENSORS 6

// Accepted range for the app-set deep-sleep time (seconds): 10 s .. 24 h.
#define SAVIA_SLEEP_MIN_S 10u
#define SAVIA_SLEEP_MAX_S 86400u
// Minimum capture cadence (the AquaCheck probe needs >= 60 s between reads).
#define SAVIA_CAPTURE_MIN_S 60u

typedef enum {
    SENSOR_NONE = 0,
    SENSOR_SDI12_AQUACHECK,   // AquaCheck sub-surface probe (SDI-12)
    SENSOR_SDI12_GENERIC,     // any SDI-12 sensor
} savia_sensor_type_t;

// One configurable sensor slot: type + the GPIO it lives on. The pin is
// app-parameterizable (a Pico-era requirement) so new sensors can be added
// and assigned to free pins without reflashing.
typedef struct {
    savia_sensor_type_t type;
    uint8_t gpio;        // data line GPIO (SDI-12 via level shifter)
    char    address;     // SDI-12 address, e.g. '0'
} savia_sensor_slot_t;

typedef struct {
    // --- Power management ---
    // How long to deep-sleep between acquisition cycles. App-parameterizable.
    // The device wakes on this timeout OR on the wake button (GPIO interrupt).
    uint32_t sleep_seconds;        // low-power nap granularity (capped by the schedule)
    uint8_t  wake_button_gpio;     // button/switch that wakes from deep sleep
    bool     deep_sleep_enabled;   // app-controlled; default OFF -> stays awake/discoverable

    // --- Schedule (mandatory wakes, independent of sleep_seconds) ---
    uint32_t capture_interval_s;   // sensor capture cadence, e.g. 3600 (hourly)
    uint8_t  daily_hour;           // UTC hour (0..23) for the daily cycle / inference

    // --- Dev / diagnostics ---
    bool    mock_enabled;          // generate mock readings instead of the real sensor
    uint8_t log_level;             // 0=DEBUG, 1=INFO (runtime, see savia/log.h)

    // --- BLE auth ---
    uint8_t auth_key[SAVIA_AUTH_KEY_LEN];   // SHA256(password); all-zero = unprovisioned

    // --- Sensors (configurable pins, multi-sensor) ---
    savia_sensor_slot_t sensors[SAVIA_MAX_SENSORS];
    uint8_t sensor_count;

    // --- LoRa (Wio-E5 over UART) ---
    bool    lora_enabled;
    uint8_t lora_uart_tx_gpio;
    uint8_t lora_uart_rx_gpio;
} station_config_t;

// Fill cfg with safe development defaults.
void config_load_defaults(station_config_t *cfg);

// Persistence in flash (implemented in config_store.c -- firmware only, uses the
// Pico SDK; host builds don't link it). load() returns true if a valid saved
// record was restored into cfg; save() writes cfg to the last flash sector.
bool config_store_load(station_config_t *cfg);
void config_store_save(const station_config_t *cfg);

#endif // SAVIA_CONFIG_H
