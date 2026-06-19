// savia_c entry point + supervisor loop.
//
// The station's four responsibilities (same as savia_py): acquire -> aggregate
// (in storage) -> infer -> serve (BLE). On a microcontroller we wrap them in a
// deep-sleep cycle: wake, sample, do periodic work, sleep until the next cycle
// or the button.
#include <stdio.h>
#include "pico/stdlib.h"

#include "savia/config.h"
#include "savia/power.h"
#include "savia/sensor.h"
#include "savia/storage.h"
#include "savia/ble.h"
#include "savia/lora.h"
#include "savia/inference.h"

int main(void) {
    stdio_init_all();

    station_config_t cfg;
    config_load_defaults(&cfg);

    power_init(&cfg);
    sensor_init(&cfg);
    storage_init();
    ble_init(&cfg);
    if (cfg.lora_enabled) {
        lora_init(&cfg);
    }

    printf("savia_c up: on_device_inference=%d, sensors=%u, sleep=%us\n",
           inference_on_device(), cfg.sensor_count, cfg.sleep_seconds);

    for (;;) {
        // 1. Acquire: read every configured sensor slot, persist each reading.
        for (uint8_t i = 0; i < cfg.sensor_count; i++) {
            savia_reading_t buf[8];
            int n = sensor_measure(&cfg.sensors[i], buf, 8);
            for (int k = 0; k < n; k++) {
                storage_append_reading(&buf[k]);
            }
        }

        // 2/3. Periodic work. TODO: gate on a schedule (LoRa every N h, the
        //      daily inference ~20:00). On-device inference only on Pico 2 W;
        //      on Pico WH the app does it from the served forecast.
        if (cfg.lora_enabled) {
            lora_cycle();
        }
        ble_poll(/*budget_ms=*/5000);

        // 4. Sleep until the next cycle or the wake button.
        savia_wake_reason_t why = power_deep_sleep(&cfg);
        if (why == SAVIA_WAKE_BUTTON) {
            // Technician pressed the button: open a longer BLE service window.
            ble_poll(/*budget_ms=*/30000);
        }
    }
}
