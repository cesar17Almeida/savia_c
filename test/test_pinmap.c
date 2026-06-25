// Host-side unit test for the GPIO inventory (pinmap). Pure logic, no SDK/HW:
// it checks the capability table, the live free/in-use/reserved state derived
// from a config, and the assignment validator the config write-path will use.
#include <assert.h>
#include <stdio.h>
#include "savia/config.h"
#include "savia/pinmap.h"

int main(void) {
    station_config_t cfg;
    config_load_defaults(&cfg);   // 1 sensor on GP2, wake on GP15, LoRa OFF (GP4/5)

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
    cfg.lora_enabled = true;   // tx=GP16, rx=GP17 by default (UART0, field wiring)
    pinmap_build(&cfg, pins);
    assert(pins[16].state == SAVIA_PIN_RESERVED &&
           pins[16].reason == SAVIA_PIN_REASON_LORA_UART);
    assert(pins[17].state == SAVIA_PIN_RESERVED);
    assert(pinmap_check_assign(&cfg, 16, pio, -1) == SAVIA_PIN_ASSIGN_RESERVED);

    // --- atomic multi-sensor validation (the sensors[] write-path gate) ---
    cfg.lora_enabled = false;   // back to GP16/17 free for these cases
    {
        savia_sensor_slot_t set[2] = {0};
        set[0].type = SENSOR_SDI12_GENERIC; set[0].gpio = 6;     // PIO ok
        set[1].type = SENSOR_ANALOG_LINEAR; set[1].gpio = 26;    // ADC ok
        int bad = 99;
        assert(pinmap_check_sensors(&cfg, set, 2, &bad) == SAVIA_PIN_ASSIGN_OK && bad == -1);

        set[1].gpio = 7;        // analog on a non-ADC pin (GP7) -> INCAPABLE at slot 1
        assert(pinmap_check_sensors(&cfg, set, 2, &bad) == SAVIA_PIN_ASSIGN_INCAPABLE && bad == 1);

        set[1].type = SENSOR_SDI12_GENERIC; set[1].gpio = 6;     // two slots, same pin
        assert(pinmap_check_sensors(&cfg, set, 2, &bad) == SAVIA_PIN_ASSIGN_OCCUPIED && bad == 0);

        set[0].gpio = 15; set[1].type = SENSOR_NONE;             // wake button is reserved
        assert(pinmap_check_sensors(&cfg, set, 2, &bad) == SAVIA_PIN_ASSIGN_RESERVED && bad == 0);
    }

    printf("test_pinmap: OK (caps + state + assignment + atomic sensors[] validation)\n");
    return 0;
}
