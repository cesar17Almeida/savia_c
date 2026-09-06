// Host-side unit test for the config module. Compiles NATIVELY (no Pico SDK, no
// hardware) because config.c is SDK-free pure logic. This is how we test the
// firmware's pure logic on the PC -- the same idea as savia_py's pytest suite.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "savia/config.h"

int main(void) {
    station_config_t cfg;
    config_load_defaults(&cfg);

    // Identity: default advertised name.
    assert(strcmp(cfg.ble_name, "Savia") == 0);

    // Power defaults (Pico-era requirements: sleep time + wake button).
    assert(cfg.sleep_seconds == 3600);   // 1 h, aligned with hourly capture
    assert(cfg.wake_button_gpio == 15);
    assert(cfg.capture_interval_s == 3600);
    assert(cfg.daily_hour == 20 && cfg.daily_min == 0);   // 20:00 LOCAL
    assert(cfg.mock_enabled == false);   // mock OFF by default; only the client enables it
    assert(cfg.log_level == 1);

    // Empty sensor table.
    assert(config_sensor_count(&cfg) == 0);          // fresh station: the app declares the table

    // Every slot zeroed; LoRa off by default, pins on the field wiring (UART0).
    assert(cfg.sensors[0].type == SENSOR_NONE);
    assert(cfg.sensors[1].type == SENSOR_NONE);
    assert(cfg.lora_enabled == false);  // off out of the box: the installer wires it from the app
    assert(cfg.lora_uart_tx_gpio == SAVIA_GPIO_NONE && cfg.lora_uart_rx_gpio == SAVIA_GPIO_NONE);
    assert(cfg.lora_period_s == 3600);      // 1 h default LoRa cycle
    assert(cfg.lora_last_signal_ms == 0);   // no signal persisted yet

    // Mode + schedule extensions (v7): FORWARD by default, local-time fields,
    // no coords until the installer sets them.
    // Host builds carry no model (SAVIA_ON_DEVICE_INFERENCE undefined), so the
    // default stays FORWARD here; a pico2_w build defaults to LOCAL.
    assert(cfg.inference_mode == SAVIA_INFER_FORWARD);
    assert(cfg.utc_offset_min == 0);
    assert(cfg.has_coords == false);
    // Slot extensions: gpio2 unused on every slot (0 would mean GP0), empty unit.
    for (int i = 0; i < SAVIA_MAX_SENSORS; i++) {
        assert(cfg.sensors[i].gpio2 == SAVIA_GPIO_NONE);
        assert(cfg.sensors[i].unit[0] == 0);
    }

    printf("test_config: OK (config defaults)\n");
    return 0;
}
