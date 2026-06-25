// GPIO inventory: per-pin silicon capabilities (static) + live state derived from
// the running config (free / in-use / system-reserved). The app reads this to
// show which pins it may assign, and the config write-path validates against it
// so a sensor can never claim a reserved pin (wireless / wake button / LoRa UART).
// Pure logic, SDK-free -> host-testable like config.c.
#ifndef SAVIA_PINMAP_H
#define SAVIA_PINMAP_H

#include <stdint.h>
#include <stdbool.h>
#include "savia/config.h"

// GP0..GP29 as exposed on the Pico W / Pico 2 W headers (both CYW43-based boards).
#define SAVIA_GPIO_COUNT 30

// What a pin can be muxed to. I2C/SPI/UART mean "this pin is muxable to that
// peripheral class"; the concrete instance and SDA/SCL/TX/RX pairing is validated
// only when a bus is actually configured (Tier B). ADC is the hard constraint:
// real analog input lives only on GP26..GP28 (GP29 is reserved for VSYS sense).
typedef enum {
    SAVIA_PIN_CAP_DIGITAL = 1u << 0,
    SAVIA_PIN_CAP_PIO     = 1u << 1,   // SDI-12 (and any bit-banged proto) rides PIO
    SAVIA_PIN_CAP_PWM     = 1u << 2,
    SAVIA_PIN_CAP_ADC     = 1u << 3,
    SAVIA_PIN_CAP_I2C     = 1u << 4,
    SAVIA_PIN_CAP_SPI     = 1u << 5,
    SAVIA_PIN_CAP_UART    = 1u << 6,
} savia_pin_cap_t;

typedef enum {
    SAVIA_PIN_FREE = 0,     // available to assign
    SAVIA_PIN_IN_USE,       // claimed by a configured sensor slot
    SAVIA_PIN_RESERVED,     // system: wireless chip / wake button / LoRa UART
} savia_pin_state_t;

typedef enum {
    SAVIA_PIN_REASON_NONE = 0,
    SAVIA_PIN_REASON_SENSOR,      // a sensor slot lives here (see `port`)
    SAVIA_PIN_REASON_WIRELESS,    // CYW43439 (GP23/24/25/29) -- gives us BLE
    SAVIA_PIN_REASON_WAKE_BTN,    // deep-sleep wake button
    SAVIA_PIN_REASON_LORA_UART,   // Wio-E5 TX/RX (only when LoRa is enabled)
} savia_pin_reason_t;

// Result of validating a candidate assignment.
typedef enum {
    SAVIA_PIN_ASSIGN_OK = 0,
    SAVIA_PIN_ASSIGN_OUT_OF_RANGE,  // gpio not on the header
    SAVIA_PIN_ASSIGN_INCAPABLE,     // pin can't do the function the sensor needs
    SAVIA_PIN_ASSIGN_RESERVED,      // wireless / wake button / LoRa UART
    SAVIA_PIN_ASSIGN_OCCUPIED,      // another sensor slot already sits on it
} savia_pin_assign_t;

typedef struct {
    uint8_t gpio;
    uint8_t caps;     // OR of savia_pin_cap_t
    uint8_t state;    // savia_pin_state_t
    uint8_t reason;   // savia_pin_reason_t
    uint8_t port;     // station port 1..6 when reason==SENSOR, else 0
} savia_pin_info_t;

// Static silicon capabilities of `gpio` (0 if out of range). Board-independent
// across the two supported boards (Pico W / Pico 2 W share the RP2040/RP2350 mux).
uint8_t pinmap_caps(uint8_t gpio);

// True for pins the wireless chip owns (GP23/24/25/29) -- permanently off-limits.
bool pinmap_is_system_reserved(uint8_t gpio);

// Caps a sensor type needs on its data pin (SDI-12 -> PIO; SENSOR_NONE -> 0).
uint8_t pinmap_caps_for_sensor(savia_sensor_type_t type);

// Fill `out` (SAVIA_GPIO_COUNT entries) with caps + live state from cfg. System
// reservations win over sensor claims, so a misconfigured slot can't hide them.
void pinmap_build(const station_config_t *cfg, savia_pin_info_t out[SAVIA_GPIO_COUNT]);

// Can `gpio` host a sensor needing `need` caps? `exclude_slot` (>=0) is ignored
// when scanning for occupancy, so editing a slot's own pin doesn't self-collide.
savia_pin_assign_t pinmap_check_assign(const station_config_t *cfg, uint8_t gpio,
                                       uint8_t need, int exclude_slot);

// Validate a whole proposed sensors[] set atomically against `base`'s system
// reservations AND against itself (so two new slots can't share a pin). Returns
// the first non-OK result and writes its slot index to *bad_index (-1 if OK).
// Pure: applies nothing. The config write-path calls this before committing.
savia_pin_assign_t pinmap_check_sensors(const station_config_t *base,
                                        const savia_sensor_slot_t *slots, uint8_t n,
                                        int *bad_index);

// Short token for an assign result (for the config_err ack message).
const char *pinmap_assign_str(savia_pin_assign_t r);

#endif // SAVIA_PINMAP_H
