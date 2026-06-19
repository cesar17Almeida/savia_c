// Station configuration. Defaults are compiled in; the app can override fields
// over BLE at runtime (persisted to flash) -- e.g. sleep time and sensor pins.
#ifndef SAVIA_CONFIG_H
#define SAVIA_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define SAVIA_MAX_SENSORS 6

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
    uint32_t sleep_seconds;        // e.g. 600 (10 min)
    uint8_t  wake_button_gpio;     // button/switch that wakes from deep sleep

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

#endif // SAVIA_CONFIG_H
