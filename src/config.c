#include "savia/config.h"
#include <string.h>

// Development defaults. The app overrides sleep time and sensor pins over BLE
// at runtime (persisted to flash) -- see the Pico-era requirements.
void config_load_defaults(station_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    // Power: 10 min between cycles; button on GPIO15 (to GND, active-low).
    cfg->sleep_seconds = 600;
    cfg->wake_button_gpio = 15;

    // One AquaCheck SDI-12 probe on GPIO2, address '0' (exposes 10 cm + 30 cm).
    cfg->sensors[0].type = SENSOR_SDI12_AQUACHECK;
    cfg->sensors[0].gpio = 2;
    cfg->sensors[0].address = '0';
    cfg->sensor_count = 1;

    // LoRa off by default; UART1 on GPIO4/5 when enabled.
    cfg->lora_enabled = false;
    cfg->lora_uart_tx_gpio = 4;
    cfg->lora_uart_rx_gpio = 5;
}
