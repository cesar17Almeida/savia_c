// Host-side unit test for the GPIO inventory (pinmap). Pure logic, no SDK/HW:
// it checks the capability table, the live free/in-use/reserved state derived
// from a config, and the assignment validator the config write-path will use.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "savia/config.h"
#include "savia/pinmap.h"
#include "savia/sensor_catalog.h"

int main(void) {
    station_config_t cfg;
    config_load_defaults(&cfg);   // no sensors, wake on GP15, LoRa OFF (no pins)
    // Defaults ship an empty sensor table, so place the probe the rest of this
    // test reasons about (slot 0 -> port 1) instead of inheriting it.
    cfg.sensors[0].type = SENSOR_SDI12_AQUACHECK;
    cfg.sensors[0].gpio = 2;
    cfg.sensors[0].address = '0';

    // --- static capabilities ---
    assert(pinmap_caps(2)  & SAVIA_PIN_CAP_PIO);     // SDI-12 needs PIO; GP2 has it
    assert(!(pinmap_caps(0)  & SAVIA_PIN_CAP_ADC));  // GP0 has no ADC
    assert(pinmap_caps(26) & SAVIA_PIN_CAP_ADC);     // GP26..28 do
    assert(pinmap_caps(28) & SAVIA_PIN_CAP_ADC);
    assert(pinmap_caps(30) == 0);                    // off the header
    assert(pinmap_is_system_reserved(23) && pinmap_is_system_reserved(29));
    assert(!pinmap_is_system_reserved(6));
    assert(pinmap_caps_for_sensor(SENSOR_SDI12_AQUACHECK) == SAVIA_PIN_CAP_PIO);
    assert(pinmap_caps_for_sensor(SENSOR_SDI12_GENERIC)   == SAVIA_PIN_CAP_PIO);
    assert(pinmap_caps_for_sensor(SENSOR_ONEWIRE_DS18B20) == SAVIA_PIN_CAP_PIO);  // 1-Wire rides PIO
    assert(pinmap_caps_for_sensor(SENSOR_ANALOG_LINEAR)   == SAVIA_PIN_CAP_ADC);  // ADC -> GP26..28
    assert(pinmap_caps_for_sensor(SENSOR_NONE) == 0);

    // --- live state from the default config ---
    savia_pin_info_t pins[SAVIA_GPIO_COUNT];
    pinmap_build(&cfg, pins);

    assert(pins[2].state == SAVIA_PIN_IN_USE);             // the AquaCheck slot
    assert(pins[2].reason == SAVIA_PIN_REASON_SENSOR && pins[2].port == 1);
    assert(pins[15].state == SAVIA_PIN_RESERVED &&
           pins[15].reason == SAVIA_PIN_REASON_WAKE_BTN);  // wake button
    assert(pins[23].state == SAVIA_PIN_RESERVED &&
           pins[23].reason == SAVIA_PIN_REASON_WIRELESS);  // CYW43
    assert(pins[29].state == SAVIA_PIN_RESERVED &&
           pins[29].reason == SAVIA_PIN_REASON_WIRELESS);
    assert(pins[4].state == SAVIA_PIN_FREE);               // LoRa off -> GP4/5 free
    assert(pins[5].state == SAVIA_PIN_FREE);
    assert(pins[6].state == SAVIA_PIN_FREE);

    // --- assignment validation (the write-path gate) ---
    uint8_t pio = SAVIA_PIN_CAP_PIO;
    assert(pinmap_check_assign(&cfg, 6,  pio, -1) == SAVIA_PIN_ASSIGN_OK);
    assert(pinmap_check_assign(&cfg, 2,  pio, -1) == SAVIA_PIN_ASSIGN_OCCUPIED);   // slot 0 sits there
    assert(pinmap_check_assign(&cfg, 2,  pio,  0) == SAVIA_PIN_ASSIGN_OK);         // ...but editing slot 0 is fine
    assert(pinmap_check_assign(&cfg, 23, pio, -1) == SAVIA_PIN_ASSIGN_RESERVED);   // wireless
    assert(pinmap_check_assign(&cfg, 15, pio, -1) == SAVIA_PIN_ASSIGN_RESERVED);   // wake button
    assert(pinmap_check_assign(&cfg, 30, pio, -1) == SAVIA_PIN_ASSIGN_OUT_OF_RANGE);
    assert(pinmap_check_assign(&cfg, 0, SAVIA_PIN_CAP_ADC, -1) == SAVIA_PIN_ASSIGN_INCAPABLE);
    assert(pinmap_check_assign(&cfg, 26, SAVIA_PIN_CAP_ADC, -1) == SAVIA_PIN_ASSIGN_OK);

    // --- enabling LoRa reserves its UART pins ---
    cfg.lora_enabled = true;   // the app wires the Wio-E5 on UART0 GP16/GP17
    cfg.lora_uart_tx_gpio = 16; cfg.lora_uart_rx_gpio = 17;
    pinmap_build(&cfg, pins);
    assert(pins[16].state == SAVIA_PIN_RESERVED &&
           pins[16].reason == SAVIA_PIN_REASON_LORA_UART);
    assert(pins[17].state == SAVIA_PIN_RESERVED);
    assert(pinmap_check_assign(&cfg, 16, pio, -1) == SAVIA_PIN_ASSIGN_RESERVED);

    // --- switching the radio off keeps its wired pins: the module is still
    //     soldered there. Only unassigning the pins frees them. ---
    cfg.lora_enabled = false;
    pinmap_build(&cfg, pins);
    assert(pins[16].state == SAVIA_PIN_RESERVED &&
           pins[16].reason == SAVIA_PIN_REASON_LORA_UART);
    assert(pinmap_check_assign(&cfg, 17, pio, -1) == SAVIA_PIN_ASSIGN_RESERVED);
    assert(pinmap_check_uart_pair(&cfg, 16, 17) == SAVIA_PIN_ASSIGN_OK);   // its own pins, never a self-collision
    cfg.lora_uart_tx_gpio = SAVIA_GPIO_NONE; cfg.lora_uart_rx_gpio = SAVIA_GPIO_NONE;   // unplugged
    pinmap_build(&cfg, pins);
    assert(pins[16].state == SAVIA_PIN_FREE && pins[17].state == SAVIA_PIN_FREE);

    // --- atomic multi-sensor validation (the sensors[] write-path gate) ---
    {
        savia_sensor_slot_t set[SAVIA_MAX_SENSORS] = {0};
        set[0].type = SENSOR_SDI12_GENERIC; set[0].gpio = 6;     // PIO ok
        set[1].type = SENSOR_ANALOG_LINEAR; set[1].gpio = 26;    // ADC ok
        int bad = 99;
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OK && bad == -1);

        set[1].gpio = 7;        // analog on a non-ADC pin (GP7) -> INCAPABLE at slot 1
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_INCAPABLE && bad == 1);

        set[1].type = SENSOR_SDI12_GENERIC; set[1].gpio = 6;     // two slots, same pin
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OCCUPIED && bad == 0);

        set[0].gpio = 15; set[1].type = SENSOR_NONE;             // wake button is reserved
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_RESERVED && bad == 0);
    }

    // --- two-pin slots (HC-SR04 trigger + echo via gpio2) ---
    {
        savia_sensor_slot_t set[SAVIA_MAX_SENSORS] = {0};
        set[0].type = SENSOR_HCSR04; set[0].gpio = 6; set[0].gpio2 = 7;
        set[1].type = SENSOR_NONE;   set[1].gpio2 = SAVIA_GPIO_NONE;
        int bad = 99;
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OK);

        set[0].gpio2 = SAVIA_GPIO_NONE;                          // echo pin missing
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OUT_OF_RANGE && bad == 0);

        set[0].gpio2 = 6;                                        // echo == trigger
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OCCUPIED && bad == 0);

        set[0].gpio2 = 15;                                       // echo on the wake button
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_RESERVED && bad == 0);

        set[0].gpio2 = 7;                                        // another slot on the echo pin
        set[1].type = SENSOR_DHT11; set[1].gpio = 7;
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OCCUPIED);

        set[1].gpio = 8;                                         // clean set: HC-SR04 + DHT11
        assert(pinmap_check_sensors(&cfg, set, &bad) == SAVIA_PIN_ASSIGN_OK);

        // pinmap_build marks BOTH pins of the two-pin slot with the same port.
        station_config_t two = cfg;
        two.sensors[0] = set[0];
        savia_pin_info_t pins2[SAVIA_GPIO_COUNT];
        pinmap_build(&two, pins2);
        assert(pins2[6].state == SAVIA_PIN_IN_USE && pins2[6].port == 1);
        assert(pins2[7].state == SAVIA_PIN_IN_USE && pins2[7].port == 1);

        // Actuator: plain DIGITAL single pin.
        savia_sensor_slot_t act[SAVIA_MAX_SENSORS] = {0};
        act[0].type = SENSOR_ACTUATOR_DIGITAL; act[0].gpio = 9;
        act[0].gpio2 = SAVIA_GPIO_NONE;
        assert(pinmap_check_sensors(&cfg, act, &bad) == SAVIA_PIN_ASSIGN_OK);
    }

    // --- catalog: every type declares its pins/caps and round-trips its token ---
    // (a missing row is already a build error; this checks the rows are sane)
    for (int t = SENSOR_NONE + 1; t < SAVIA_SENSOR_TYPE_COUNT; t++) {
        savia_sensor_type_t type = (savia_sensor_type_t) t;
        const char *token = sensor_type_token(type);
        assert(token[0] != '\0');
        assert(sensor_type_from_token(token, strlen(token)) == type);   // round-trip
        assert(sensor_type_caps(type) != 0);                            // needs SOME pin
        uint8_t pins_used = sensor_type_pins(type);
        assert(pins_used == 1 || pins_used == 2);
    }
    assert(sensor_type_from_token("nope", 4) == SENSOR_NONE);           // unknown -> empty
    assert(sensor_type_pins(SENSOR_HCSR04) == 2);                       // trigger + echo
    assert(sensor_type_extra(SENSOR_ANALOG_LINEAR) ==
           (SAVIA_SENSOR_KIND_DEPTH | SAVIA_SENSOR_SCALE_OFFSET));
    assert(sensor_type_extra(SENSOR_SDI12_GENERIC) == SAVIA_SENSOR_CHANNELS);
    assert(sensor_type_extra(SENSOR_SDI12_AQUACHECK) == 0);             // fixed layout

    printf("test_pinmap: OK (caps + state + assignment + atomic sensors[] + gpio2 + catalog)\n");
    // --- LoRa UART pair validator ---
    assert(pinmap_check_uart_pair(&cfg, 16, 17) == SAVIA_PIN_ASSIGN_OK);          // uart0 pair, free
    assert(pinmap_check_uart_pair(&cfg, 0, 17)  == SAVIA_PIN_ASSIGN_OK);          // cross pair, same uart0
    assert(pinmap_check_uart_pair(&cfg, 4, 17)  == SAVIA_PIN_ASSIGN_INCAPABLE);   // uart1 TX with uart0 RX
    assert(pinmap_check_uart_pair(&cfg, 17, 16) == SAVIA_PIN_ASSIGN_INCAPABLE);   // roles swapped
    assert(pinmap_check_uart_pair(&cfg, 12, 15) == SAVIA_PIN_ASSIGN_INCAPABLE);   // GP15 is not a UART RX
    assert(pinmap_check_uart_pair(&cfg, 28, 29) == SAVIA_PIN_ASSIGN_RESERVED);    // GP29 belongs to the radio
    assert(pinmap_check_uart_pair(&cfg, 0, 1)   == SAVIA_PIN_ASSIGN_OK);
    cfg.sensors[1].type = SENSOR_ONEWIRE_DS18B20; cfg.sensors[1].gpio = 1;
    assert(pinmap_check_uart_pair(&cfg, 0, 1)   == SAVIA_PIN_ASSIGN_OCCUPIED);    // a sensor sits on GP1
    cfg.sensors[1].type = SENSOR_NONE;
    cfg.lora_enabled = true; cfg.lora_uart_tx_gpio = 16; cfg.lora_uart_rx_gpio = 17;
    assert(pinmap_check_uart_pair(&cfg, 16, 17) == SAVIA_PIN_ASSIGN_OK);          // keeping its own pins is fine
    assert(pinmap_check_uart_pair(&cfg, 20, 21) == SAVIA_PIN_ASSIGN_OK);          // moving to uart1
    assert(pinmap_check_uart_pair(&cfg, 40, 41) == SAVIA_PIN_ASSIGN_OUT_OF_RANGE);
    cfg.lora_enabled = false;

    return 0;
}
