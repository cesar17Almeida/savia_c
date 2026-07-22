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

    // Schedule: capture hourly, daily cycle (inference) at 20:00 LOCAL time --
    // Antonio's forecast loop starts at 20:00. Irrigation at 06:00 (informative).
    // utc_offset_min 0 until the app/backend sets it (local == UTC out of the box).
    cfg->capture_interval_s = 3600;
    cfg->daily_hour = 20;
    cfg->utc_offset_min = 0;
    cfg->irrigation_hour = 6;

    // Mode: FORWARD (data store) -- safe on any board; the app can switch to LOCAL
    // on builds where inference_on_device() is true.
    cfg->inference_mode = SAVIA_INFER_FORWARD;
    cfg->has_coords = false;       // installer sets coords from the app

    // Mock data OFF by default -- the station reads the real sensor out of the box.
    // Only the client (TerraLink over BLE) may turn mock on; see ble_gatt config write.
    cfg->mock_enabled = false;
    cfg->log_level = 1;            // SAVIA_LOG_INFO

    // AquaCheck SDI-12 probe, addr '0'. Default pin GP2; the real pin is set from
    // the app (bring-up wired it to GP18). Real probe = SKU 1120-0404: 4 sensors at
    // 10/20/30/40 cm, order top->bottom (HS10=value[0], HS30=value[2]).
    // Verified by bring-up; see tools/sdi12_bringup/AQUACHECK_RESPONSES.md.
    cfg->sensors[0].type = SENSOR_SDI12_AQUACHECK;
    cfg->sensors[0].gpio = 2;
    cfg->sensors[0].address = '0';
    cfg->sensor_count = 1;
    // gpio2 is "unused" (0xFF) on every slot; memset(0) above would leave 0 = GP0.
    for (int i = 0; i < SAVIA_MAX_SENSORS; i++) cfg->sensors[i].gpio2 = SAVIA_GPIO_NONE;

    // LoRa off by default (the app enables it / pings on demand). Default pins are
    // the field wiring: Wio-E5 on UART0 GP16(TX)/GP17(RX). Last-signal fields zeroed.
    cfg->lora_enabled = false;
    cfg->lora_uart_tx_gpio = 16;
    cfg->lora_uart_rx_gpio = 17;
    cfg->lora_period_s = 3600;   // 1 h; a private gateway + paid plan lift the TTN FUP
}
