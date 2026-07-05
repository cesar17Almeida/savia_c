#include "savia/pinmap.h"

// GPIO inventory for the RP2040 / RP2350 as wired on the Pico W / Pico 2 W.
// Capabilities are static silicon facts; state is derived from the live config.
// SDK-free on purpose (just a table + logic) so it unit-tests on the host.

// Pins the CYW43439 wireless chip owns -- and that chip is what gives us BLE, so
// these are permanently off-limits regardless of config:
//   GP23 WL_ON, GP24 WL_DATA, GP25 WL_CS, GP29 WL_CLK / VSYS-sense (ADC3).
static bool is_wireless_pin(uint8_t gpio) {
    return gpio == 23 || gpio == 24 || gpio == 25 || gpio == 29;
}

uint8_t pinmap_caps(uint8_t gpio) {
    if (gpio >= SAVIA_GPIO_COUNT) return 0;
    // Every user GPIO can be muxed to digital/PIO/PWM and to the bus peripherals
    // (instance/pair pinning is checked later, at bus-assign time).
    uint8_t caps = SAVIA_PIN_CAP_DIGITAL | SAVIA_PIN_CAP_PIO | SAVIA_PIN_CAP_PWM |
                   SAVIA_PIN_CAP_I2C | SAVIA_PIN_CAP_SPI | SAVIA_PIN_CAP_UART;
    // Real analog input only on the ADC channels GP26..GP29.
    if (gpio >= 26 && gpio <= 29) caps |= SAVIA_PIN_CAP_ADC;
    return caps;
}

bool pinmap_is_system_reserved(uint8_t gpio) {
    return is_wireless_pin(gpio);
}

uint8_t pinmap_caps_for_sensor(savia_sensor_type_t type) {
    switch (type) {
        case SENSOR_SDI12_AQUACHECK:
        case SENSOR_SDI12_GENERIC:
        case SENSOR_ONEWIRE_DS18B20:
            return SAVIA_PIN_CAP_PIO;   // SDI-12 / 1-Wire framing is driven by a PIO program
        case SENSOR_ANALOG_LINEAR:
            return SAVIA_PIN_CAP_ADC;   // real analog input -> GP26..GP28 only
        case SENSOR_DHT11:              // bit-banged proprietary 1-wire timing
        case SENSOR_HCSR04:             // trigger/echo pulses (both pins DIGITAL)
        case SENSOR_ACTUATOR_DIGITAL:   // plain GPIO output
            return SAVIA_PIN_CAP_DIGITAL;
        default:
            return 0;
    }
}

// Second data pin, if the type uses one (HC-SR04 echo). SAVIA_GPIO_NONE = none.
static uint8_t slot_gpio2(const savia_sensor_slot_t *s) {
    return s->type == SENSOR_HCSR04 ? s->gpio2 : SAVIA_GPIO_NONE;
}

void pinmap_build(const station_config_t *cfg, savia_pin_info_t out[SAVIA_GPIO_COUNT]) {
    for (uint8_t g = 0; g < SAVIA_GPIO_COUNT; g++) {
        out[g].gpio   = g;
        out[g].caps   = pinmap_caps(g);
        out[g].state  = SAVIA_PIN_FREE;
        out[g].reason = SAVIA_PIN_REASON_NONE;
        out[g].port   = 0;
        if (is_wireless_pin(g)) {
            out[g].state  = SAVIA_PIN_RESERVED;
            out[g].reason = SAVIA_PIN_REASON_WIRELESS;
        }
    }
    if (!cfg) return;

    // System reservations win over sensor claims -> apply them first.
    if (cfg->wake_button_gpio < SAVIA_GPIO_COUNT &&
        out[cfg->wake_button_gpio].state == SAVIA_PIN_FREE) {
        out[cfg->wake_button_gpio].state  = SAVIA_PIN_RESERVED;
        out[cfg->wake_button_gpio].reason = SAVIA_PIN_REASON_WAKE_BTN;
    }
    if (cfg->lora_enabled) {
        uint8_t lp[2] = { cfg->lora_uart_tx_gpio, cfg->lora_uart_rx_gpio };
        for (int i = 0; i < 2; i++) {
            if (lp[i] < SAVIA_GPIO_COUNT && out[lp[i]].state == SAVIA_PIN_FREE) {
                out[lp[i]].state  = SAVIA_PIN_RESERVED;
                out[lp[i]].reason = SAVIA_PIN_REASON_LORA_UART;
            }
        }
    }

    // Sensors claim only still-free pins (validation keeps it that way at runtime).
    // A slot may own TWO pins (HC-SR04 trigger + echo): both carry the same port.
    uint8_t n = cfg->sensor_count <= SAVIA_MAX_SENSORS ? cfg->sensor_count : SAVIA_MAX_SENSORS;
    for (uint8_t i = 0; i < n; i++) {
        if (cfg->sensors[i].type == SENSOR_NONE) continue;
        uint8_t pins[2] = { cfg->sensors[i].gpio, slot_gpio2(&cfg->sensors[i]) };
        for (int p = 0; p < 2; p++) {
            uint8_t g = pins[p];
            if (g < SAVIA_GPIO_COUNT && out[g].state == SAVIA_PIN_FREE) {
                out[g].state  = SAVIA_PIN_IN_USE;
                out[g].reason = SAVIA_PIN_REASON_SENSOR;
                out[g].port   = (uint8_t)(i + 1);
            }
        }
    }
}

savia_pin_assign_t pinmap_check_assign(const station_config_t *cfg, uint8_t gpio,
                                       uint8_t need, int exclude_slot) {
    if (gpio >= SAVIA_GPIO_COUNT) return SAVIA_PIN_ASSIGN_OUT_OF_RANGE;
    if ((pinmap_caps(gpio) & need) != need) return SAVIA_PIN_ASSIGN_INCAPABLE;
    if (is_wireless_pin(gpio)) return SAVIA_PIN_ASSIGN_RESERVED;
    if (!cfg) return SAVIA_PIN_ASSIGN_OK;
    if (gpio == cfg->wake_button_gpio) return SAVIA_PIN_ASSIGN_RESERVED;
    if (cfg->lora_enabled &&
        (gpio == cfg->lora_uart_tx_gpio || gpio == cfg->lora_uart_rx_gpio)) {
        return SAVIA_PIN_ASSIGN_RESERVED;
    }
    uint8_t n = cfg->sensor_count <= SAVIA_MAX_SENSORS ? cfg->sensor_count : SAVIA_MAX_SENSORS;
    for (uint8_t i = 0; i < n; i++) {
        if ((int) i == exclude_slot) continue;
        if (cfg->sensors[i].type == SENSOR_NONE) continue;
        if (cfg->sensors[i].gpio == gpio) return SAVIA_PIN_ASSIGN_OCCUPIED;
        if (slot_gpio2(&cfg->sensors[i]) == gpio) return SAVIA_PIN_ASSIGN_OCCUPIED;
    }
    return SAVIA_PIN_ASSIGN_OK;
}

savia_pin_assign_t pinmap_check_sensors(const station_config_t *base,
                                        const savia_sensor_slot_t *slots, uint8_t n,
                                        int *bad_index) {
    if (bad_index) *bad_index = -1;
    if (n > SAVIA_MAX_SENSORS) n = SAVIA_MAX_SENSORS;

    // Validate the proposed set against a scratch config that keeps `base`'s system
    // reservations (wake button / LoRa UART) but swaps in the NEW sensors -- so a
    // pin shared by two of the new slots is caught as OCCUPIED, not silently kept.
    station_config_t scratch = *base;
    scratch.sensor_count = n;
    for (uint8_t i = 0; i < n; i++) scratch.sensors[i] = slots[i];
    for (uint8_t i = n; i < SAVIA_MAX_SENSORS; i++) scratch.sensors[i].type = SENSOR_NONE;

    for (uint8_t i = 0; i < n; i++) {
        if (slots[i].type == SENSOR_NONE) continue;
        uint8_t need = pinmap_caps_for_sensor(slots[i].type);
        savia_pin_assign_t r = pinmap_check_assign(&scratch, slots[i].gpio, need, (int) i);
        if (r != SAVIA_PIN_ASSIGN_OK) {
            if (bad_index) *bad_index = (int) i;
            return r;
        }
        // Two-pin types: gpio2 must exist (HC-SR04 without echo -> out of range,
        // since SAVIA_GPIO_NONE = 0xFF), differ from gpio, and pass the same checks.
        if (slots[i].type == SENSOR_HCSR04) {
            if (slots[i].gpio2 == slots[i].gpio) {
                if (bad_index) *bad_index = (int) i;
                return SAVIA_PIN_ASSIGN_OCCUPIED;
            }
            r = pinmap_check_assign(&scratch, slots[i].gpio2, need, (int) i);
            if (r != SAVIA_PIN_ASSIGN_OK) {
                if (bad_index) *bad_index = (int) i;
                return r;
            }
        }
    }
    return SAVIA_PIN_ASSIGN_OK;
}

const char *pinmap_assign_str(savia_pin_assign_t r) {
    switch (r) {
        case SAVIA_PIN_ASSIGN_OK:           return "ok";
        case SAVIA_PIN_ASSIGN_OUT_OF_RANGE: return "gpio out of range";
        case SAVIA_PIN_ASSIGN_INCAPABLE:    return "pin incapable";
        case SAVIA_PIN_ASSIGN_RESERVED:     return "pin reserved";
        case SAVIA_PIN_ASSIGN_OCCUPIED:     return "pin occupied";
        default:                            return "?";
    }
}
