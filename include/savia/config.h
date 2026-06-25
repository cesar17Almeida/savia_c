// Station configuration. Defaults are compiled in; the app can override fields
// over BLE at runtime (persisted to flash) -- e.g. sleep time and sensor pins.
#ifndef SAVIA_CONFIG_H
#define SAVIA_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "savia/auth.h"

#define SAVIA_MAX_SENSORS 6

// How many returned values a generic SDI-12 sensor can be mapped to: the installer
// labels each value index -> {kind, depth}. Kept small so the slot stays bounded
// (the whole config must fit one 256 B flash page).
#define SAVIA_SDI12_MAX_CHANNELS 4

// Max advertised BLE name (incl. NUL). The 128-bit service UUID moves to the scan
// response, so the ADV packet leaves room for a name up to 26 chars; cap at 20.
#define SAVIA_BLE_NAME_MAX 21

// Accepted range for the app-set deep-sleep time (seconds): 10 s .. 24 h.
#define SAVIA_SLEEP_MIN_S 10u
#define SAVIA_SLEEP_MAX_S 86400u
// Minimum capture cadence (the AquaCheck probe needs >= 60 s between reads).
#define SAVIA_CAPTURE_MIN_S 60u

// Sensor catalog. A sensor is interface + protocol + decoding -- a free GPIO is
// necessary but not sufficient -- so the app picks a TYPE the firmware supports
// and assigns it to a capable free pin. Three molds make "any sensor" honest:
//   - SDI-12 (self-describing bus): AquaCheck has a fixed layout; "generic" works
//     for any SDI-12 sensor by letting the installer label each returned value.
//   - analog linear (the cheap dumb sensor): installer supplies scale/offset.
//   - 1-Wire DS18B20 (cheap digital thermometer): self-identifying, degrees C.
// See docs/SENSORS.md.
typedef enum {
    SENSOR_NONE = 0,
    SENSOR_SDI12_AQUACHECK,    // AquaCheck sub-surface probe (SDI-12), fixed layout
    SENSOR_SDI12_GENERIC,      // any SDI-12 sensor; installer maps each value index
    SENSOR_ANALOG_LINEAR,      // dumb analog on an ADC pin: value = scale*raw + offset
    SENSOR_ONEWIRE_DS18B20,    // 1-Wire digital thermometer (self-identifying, degC)
} savia_sensor_type_t;

// Installer-supplied meaning of one returned value: which reading kind it is and,
// for soil sensors, at what depth. Used per value index by SDI-12 generic, and as
// the single semantic for analog / 1-Wire.
typedef struct {
    uint8_t kind;      // savia_reading_kind_t
    uint8_t depth_cm;  // 0 for non-soil (air / bare temperature)
} savia_channel_t;

// One configurable sensor slot: a free GPIO + a sensor type + the decoding the
// firmware can't infer on its own. The pin is app-parameterizable (a Pico-era
// requirement) so sensors can be added/moved without reflashing. `map` carries the
// type-specific decoding; `kind`/`depth_cm` are the default semantics for the
// single value returned by analog / 1-Wire.
typedef struct {
    savia_sensor_type_t type;
    uint8_t gpio;        // data GPIO (SDI-12/1-Wire via level shifter; analog = ADC pin)
    char    address;     // SDI-12 address, e.g. '0' (ignored for analog / 1-Wire)
    uint8_t kind;        // savia_reading_kind_t for the analog / 1-Wire value
    uint8_t depth_cm;    // depth for that value (0 = none)
    union {
        struct { float scale, offset; } analog;   // SENSOR_ANALOG_LINEAR
        struct {                                   // SENSOR_SDI12_GENERIC
            uint8_t count;
            savia_channel_t ch[SAVIA_SDI12_MAX_CHANNELS];
        } sdi12;
    } map;
} savia_sensor_slot_t;

typedef struct {
    // --- Identity ---
    char ble_name[SAVIA_BLE_NAME_MAX];   // advertised BLE name; app-editable (default "Savia")

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

    // --- LoRa last-known downlink signal (persisted runtime state, NOT a user
    // setting; not in the …0013 snapshot -- served via status …0010 and seeded
    // into the driver at boot so the app shows it after a reboot). ---
    int16_t  lora_last_rssi_dbm;
    int16_t  lora_last_snr_ddb;    // deci-dB (x10)
    uint64_t lora_last_signal_ms;  // wall-clock ms of the last signal (0 = never)
} station_config_t;

// Fill cfg with safe development defaults.
void config_load_defaults(station_config_t *cfg);

// Persistence in flash (implemented in config_store.c -- firmware only, uses the
// Pico SDK; host builds don't link it). load() returns true if a valid saved
// record was restored into cfg; save() writes cfg to the last flash sector.
bool config_store_load(station_config_t *cfg);
void config_store_save(const station_config_t *cfg);

#endif // SAVIA_CONFIG_H
