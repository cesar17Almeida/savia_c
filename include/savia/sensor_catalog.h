// The sensor catalog: ONE row per supported sensor type, and the only place the
// firmware describes what a type is. Everything that used to switch on
// savia_sensor_type_t reads this list -- pin capabilities (pinmap), wire token
// (BLE codec), which extra config fields the type carries (config snapshot), how
// many pins it takes, and which function measures it.
//
// Adding a sensor is one row here plus its measure() in the driver. Forgetting the
// row is a compile error (see the _Static_assert in sensor_catalog.c).
//
//   X(enum, wire token, needed pin caps, pins used, role, extra config fields, measure fn)
//
// Why a macro list and not a plain struct table: the last column names a function
// that only exists in the driver (which needs the Pico SDK), while every other
// column is consumed by SDK-free code that must also compile natively for the host
// tests. Each side expands the list and ignores the columns it doesn't use.
#ifndef SAVIA_SENSOR_CATALOG_H
#define SAVIA_SENSOR_CATALOG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "savia/config.h"
#include "savia/pinmap.h"

// What a slot is for. An INPUT measures and stores readings on a cadence; an
// OUTPUT is driven by the supervisor and never produces a reading, so it must
// never schedule a capture nor bound the sleep (see scheduler.c).
typedef enum {
    SAVIA_SLOT_INPUT  = 0,
    SAVIA_SLOT_OUTPUT = 1,
} savia_slot_role_t;

// Extra per-slot config a type carries beyond {gpio, type, addr, interval_s}. The
// BLE config snapshot emits exactly these fields, and the patch parser commits the
// matching union arm.
typedef enum {
    SAVIA_SENSOR_KIND_DEPTH    = 1u << 0,   // kind + depth_cm (analog / 1-Wire)
    SAVIA_SENSOR_SCALE_OFFSET  = 1u << 1,   // map.analog {scale, offset}
    SAVIA_SENSOR_CHANNELS      = 1u << 2,   // map.sdi12 chan[] (installer-labelled)
} savia_sensor_extra_t;

// PIO for the bit-banged one-wire protocols (SDI-12, 1-Wire), ADC for real analog
// input (GP26..28 only), DIGITAL for plain pulses and outputs.
#define SAVIA_SENSOR_CATALOG(X)                                                                       \
    /*   type,                    token,             caps,                  pins, role,              extra,              measure     */ \
    X(SENSOR_SDI12_AQUACHECK,  "sdi12_aquacheck",  SAVIA_PIN_CAP_PIO,     1, SAVIA_SLOT_INPUT,  0,                                      measure_sdi12)   \
    X(SENSOR_SDI12_GENERIC,    "sdi12_generic",    SAVIA_PIN_CAP_PIO,     1, SAVIA_SLOT_INPUT,  SAVIA_SENSOR_CHANNELS,                  measure_sdi12)   \
    X(SENSOR_ANALOG_LINEAR,    "analog_linear",    SAVIA_PIN_CAP_ADC,     1, SAVIA_SLOT_INPUT,  SAVIA_SENSOR_KIND_DEPTH |                                \
                                                                                                SAVIA_SENSOR_SCALE_OFFSET,              measure_analog)  \
    X(SENSOR_ONEWIRE_DS18B20,  "onewire_ds18b20",  SAVIA_PIN_CAP_PIO,     1, SAVIA_SLOT_INPUT,  SAVIA_SENSOR_KIND_DEPTH,                measure_ds18b20) \
    X(SENSOR_DHT11,            "dht11",            SAVIA_PIN_CAP_DIGITAL, 1, SAVIA_SLOT_INPUT,  0,                                      measure_dht11)   \
    X(SENSOR_HCSR04,           "hc_sr04",          SAVIA_PIN_CAP_DIGITAL, 2, SAVIA_SLOT_INPUT,  0,                                      measure_hcsr04)  \
    X(SENSOR_ACTUATOR_DIGITAL, "actuator",         SAVIA_PIN_CAP_DIGITAL, 1, SAVIA_SLOT_OUTPUT, 0,                                      measure_none)

// Wire token for `type` ("none" when it has no row, matching an empty slot).
const char *sensor_type_token(savia_sensor_type_t type);

// Inverse, over a NON NUL-terminated CBOR text slice. Unknown -> SENSOR_NONE.
savia_sensor_type_t sensor_type_from_token(const char *s, size_t n);

// Pin capabilities the type needs on its data pin(s). 0 when it has no row.
uint8_t sensor_type_caps(savia_sensor_type_t type);

// Data pins the type occupies: 1, or 2 for trigger/echo pairs (gpio + gpio2).
uint8_t sensor_type_pins(savia_sensor_type_t type);

// OR of savia_sensor_extra_t: the extra config fields this type carries.
uint8_t sensor_type_extra(savia_sensor_type_t type);

// True for output-only slots (actuators): driven, never measured. Unknown -> false.
bool sensor_type_is_output(savia_sensor_type_t type);

#endif // SAVIA_SENSOR_CATALOG_H
