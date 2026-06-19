// Host-side unit test for the config module. Compiles NATIVELY (no Pico SDK, no
// hardware) because config.c is SDK-free pure logic. This is how we test the
// firmware's pure logic on the PC -- the same idea as savia_py's pytest suite.
#include <assert.h>
#include <stdio.h>
#include "savia/config.h"

int main(void) {
    station_config_t cfg;
    config_load_defaults(&cfg);

    // Power defaults (Pico-era requirements: sleep time + wake button).
    assert(cfg.sleep_seconds == 600);
    assert(cfg.wake_button_gpio == 15);

    // One AquaCheck SDI-12 sensor on a configurable pin.
    assert(cfg.sensor_count == 1);
    assert(cfg.sensors[0].type == SENSOR_SDI12_AQUACHECK);
    assert(cfg.sensors[0].gpio == 2);
    assert(cfg.sensors[0].address == '0');

    // Remaining slots zeroed; LoRa off by default.
    assert(cfg.sensors[1].type == SENSOR_NONE);
    assert(cfg.lora_enabled == false);

    printf("test_config: OK (config defaults)\n");
    return 0;
}
