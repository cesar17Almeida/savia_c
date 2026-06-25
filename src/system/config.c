#include "savia/config.h"
#include <string.h>

// Development defaults. The app overrides sleep time and sensor pins over BLE
// at runtime (persisted to flash) -- see the Pico-era requirements.
void config_load_defaults(station_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    // Identity: advertised name (app-editable).
    strncpy(cfg->ble_name, "Savia", SAVIA_BLE_NAME_MAX - 1);

    // Power: 10 min between cycles; button on GPIO15 (to GND, active-low).
    // Deep sleep is OFF by default -- the device stays awake and discoverable
    // until the app enables it; only then does it power the radio down to sleep.
    cfg->sleep_seconds = 3600;     // default nap = 1 h, aligned with hourly capture
    cfg->wake_button_gpio = 15;
    cfg->deep_sleep_enabled = false;

    // Schedule: capture hourly, daily cycle at 20:00 UTC (overrides a long sleep).
    cfg->capture_interval_s = 3600;
    cfg->daily_hour = 20;

    // Dev: mock data ON during bring-up; INFO logs.
    cfg->mock_enabled = true;
    cfg->log_level = 1;            // SAVIA_LOG_INFO

    // One AquaCheck SDI-12 probe on GPIO2, address '0' (exposes 10 cm + 30 cm).
    cfg->sensors[0].type = SENSOR_SDI12_AQUACHECK;
    cfg->sensors[0].gpio = 2;
    cfg->sensors[0].address = '0';
    cfg->sensor_count = 1;

    // LoRa off by default (the app enables it / pings on demand). Default pins are
    // the field wiring: Wio-E5 on UART0 GP16(TX)/GP17(RX). Last-signal fields zeroed.
    cfg->lora_enabled = false;
    cfg->lora_uart_tx_gpio = 16;
    cfg->lora_uart_rx_gpio = 17;
}
