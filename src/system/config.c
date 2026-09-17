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
    cfg->daily_min = 0;
    cfg->utc_offset_min = 0;

    // Mode: infer HERE whenever the build carries the model (Pico 2 W); a board
    // without it can only be a data store, and the BLE write-path would reject
    // LOCAL anyway. Host builds don't define the macro -> FORWARD.
#if SAVIA_ON_DEVICE_INFERENCE
    cfg->inference_mode = SAVIA_INFER_LOCAL;
#else
    cfg->inference_mode = SAVIA_INFER_FORWARD;
#endif
    cfg->has_coords = false;       // installer sets coords from the app

    // Mock data OFF by default -- the station reads the real sensor out of the box.
    // Only the client (TerraLink over BLE) may turn mock on; see ble_gatt config write.
    // A MOCK build (make build MOCK=ON) is the exception: it always replays the dataset.
#if SAVIA_MOCK_DATA
    cfg->mock_enabled = true;
#else
    cfg->mock_enabled = false;
#endif
    cfg->log_level = 1;            // SAVIA_LOG_INFO

    // No sensors out of the box: the installer declares the whole table from the
    // app, so a fresh station is configured from scratch instead of inheriting a
    // probe it may not have. An empty table is a supported state -- the scheduler
    // just falls back to the daily wake (see scheduler_next_sleep_s).
    // The AquaCheck (SKU 1120-0404: 4 sensors at 10/20/30/40 cm, top->bottom,
    // HS10=value[0], HS30=value[2], addr '0') is added from TerraLink like any
    // other; see tools/sdi12_bringup/AQUACHECK_RESPONSES.md.
    // Every slot free (memset gave type = SENSOR_NONE); gpio2 is "unused" (0xFF),
    // which memset(0) would have left as 0 = GP0.
    for (int i = 0; i < SAVIA_MAX_SENSORS; i++) cfg->sensors[i].gpio2 = SAVIA_GPIO_NONE;

    // LoRa OFF out of the box: a fresh station has no module wired until the
    // installer enables it from TerraLink and names the UART pins (setup wizard).
    // Until then the clock comes from the phone on its first connection.
    cfg->lora_enabled = false;
    cfg->lora_uart_tx_gpio = SAVIA_GPIO_NONE;
    cfg->lora_uart_rx_gpio = SAVIA_GPIO_NONE;
    cfg->lora_period_s = 3600;   // 1 h; a private gateway + paid plan lift the TTN FUP
}

uint8_t config_sensor_count(const station_config_t *cfg) {
    uint8_t n = 0;
    for (int i = 0; i < SAVIA_MAX_SENSORS; i++) if (savia_slot_used(&cfg->sensors[i])) n++;
    return n;
}

int config_first_free_slot(const station_config_t *cfg) {
    for (int i = 0; i < SAVIA_MAX_SENSORS; i++) if (!savia_slot_used(&cfg->sensors[i])) return i;
    return -1;
}
