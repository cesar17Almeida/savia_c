// savia_c entry point + supervisor loop.
//
// The station's four responsibilities (same as savia_py): acquire -> aggregate
// (in storage) -> infer -> serve (BLE). On a microcontroller we wrap them in a
// deep-sleep cycle: wake, sample, do periodic work, sleep until the next cycle
// or the button.
#include <stdio.h>
#include "pico/stdlib.h"
#if SAVIA_ENABLE_BLE
#include "pico/cyw43_arch.h"      // cyw43_arch_lwip_begin/end == async_context lock
#endif

#include "savia/config.h"
#include "savia/power.h"
#include "savia/sensor.h"
#include "savia/storage.h"
#include "savia/clock.h"
#include "savia/ble.h"
#include "savia/lora.h"
#include "savia/inference.h"
#include "savia/scheduler.h"
#include "savia/log.h"

// Dev/mock: seed 48 h of hourly readings (HS10, HS30, TA) at boot -- the LSTM's
// past window -- so the app has a dataset before live sampling accumulates.
static void seed_mock_readings(void) {
    const uint64_t base = 1782000000000ULL;   // ~20 Jun 2026 UTC (mock baseline)
    for (int h = 48; h >= 1; h--) {
        uint64_t ts = base - (uint64_t) h * 3600000ULL;
        float drift = (float) h * 0.002f;       // older = slightly wetter (dry-down)
        savia_reading_t r10 = { .ts_ms = ts, .port = 1, .depth_cm = 10,
                                .kind = READING_SOIL_MOISTURE, .value = 0.70f + drift };
        savia_reading_t r30 = { .ts_ms = ts, .port = 1, .depth_cm = 30,
                                .kind = READING_SOIL_MOISTURE, .value = 0.74f + drift };
        savia_reading_t rta = { .ts_ms = ts, .port = 1, .depth_cm = 0,
                                .kind = READING_AIR_TEMPERATURE, .value = 20.0f + drift * 4.0f };
        storage_append_reading(&r10);
        storage_append_reading(&r30);
        storage_append_reading(&rta);
    }
}

// Serialize cfg access against the BLE write path. In threadsafe-background mode
// BTstack (incl. handle_config_write's `*g_cfg = next`) runs under the cyw43
// async_context lock; holding it here makes the ~196 B cfg copy atomic vs the
// supervisor, preventing torn reads of cfg.sensors[i]. No-op when BLE is off.
static inline void cfg_lock(void) {
#if SAVIA_ENABLE_BLE
    cyw43_arch_lwip_begin();
#endif
}
static inline void cfg_unlock(void) {
#if SAVIA_ENABLE_BLE
    cyw43_arch_lwip_end();
#endif
}

// Persist the latest LoRa downlink signal into cfg (survives reboot) if it
// advanced. Snapshots under the lock so the flash write isn't torn by a BLE write.
static void lora_persist_signal(station_config_t *cfg) {
    lora_status_t ls;
    lora_get_status(&ls);
    if (!ls.last_signal_ms || ls.last_signal_ms == cfg->lora_last_signal_ms) return;
    station_config_t snap;
    cfg_lock();
    cfg->lora_last_rssi_dbm  = ls.rssi_dbm;
    cfg->lora_last_snr_ddb   = ls.snr_ddb;
    cfg->lora_last_signal_ms = ls.last_signal_ms;
    snap = *cfg;
    cfg_unlock();
    config_store_save(&snap);
}

// Time source for log line stamps: wall-clock once synced, uptime before that.
static uint64_t log_clock(bool *wall) {
    uint64_t up = to_ms_since_boot(get_absolute_time());
    if (clock_is_set()) { *wall = true; return clock_now(up); }
    *wall = false;
    return up;
}

int main(void) {
    stdio_init_all();
    savia_log_set_clock(log_clock);   // timestamp every log line

    station_config_t cfg;
    config_load_defaults(&cfg);
    if (config_store_load(&cfg)) {
        printf("config: restored from flash (sleep_s=%u)\n", cfg.sleep_seconds);
    }
    savia_log_set_level(cfg.log_level);

    power_init(&cfg);

    // Recovery: power on with the wake button held -> factory reset (wipes the
    // password and all settings back to defaults). The reverted BLE name ("Savia")
    // is the visible confirmation.
    if (power_reset_button_held(&cfg, SAVIA_FACTORY_RESET_HOLD_MS)) {
        LOG_WARN("button held at boot -> factory reset (clearing password + config)\n");
        config_load_defaults(&cfg);
        config_store_save(&cfg);
        savia_log_set_level(cfg.log_level);
    }

    sensor_init(&cfg);
    storage_init();
    bool mock_seeded = false;
    if (cfg.mock_enabled) { seed_mock_readings(); mock_seeded = true; }   // dev dataset
    ble_init(&cfg);
    if (cfg.lora_enabled) {
        lora_init(&cfg);
    }
    // Show the last LoRa signal (from a prior power cycle) until a fresh ping.
    lora_seed_last_signal(cfg.lora_last_rssi_dbm, cfg.lora_last_snr_ddb,
                          cfg.lora_last_signal_ms);

    printf("savia_c up: on_device_inference=%d, sensors=%u, sleep=%us, capture=%us, daily_h=%u\n",
           inference_on_device(), cfg.sensor_count, cfg.sleep_seconds,
           cfg.capture_interval_s, cfg.daily_hour);

    savia_scheduler_t sched;
    scheduler_init(&sched);

    for (;;) {
        uint64_t up = to_ms_since_boot(get_absolute_time());
        bool timed = clock_is_set();
        uint64_t now_ms = timed ? clock_now(up) : up;

        // Take a consistent snapshot of the BLE-owned cfg so the rest of the
        // iteration (incl. the long SDI-12 read of cfg.sensors[i]) sees a stable
        // copy even if a BLE config write lands mid-cycle.
        station_config_t live;
        cfg_lock();
        live = cfg;
        cfg_unlock();

        // 1. Decide what's due now. The sleep time is only the low-power tick;
        //    the schedule forces the mandatory wakes. Before time is set we just
        //    capture each cycle so we never sit idle without data.
        savia_sched_action_t act = timed
            ? scheduler_tick(&sched, now_ms, live.capture_interval_s, live.daily_hour)
            : (savia_sched_action_t){ .capture = true, .daily = false };

        // Sync mock state if the app toggled it (re-seed the dataset on enable).
        if (live.mock_enabled && !mock_seeded) {
            storage_clear(); seed_mock_readings(); mock_seeded = true;
        } else if (!live.mock_enabled && mock_seeded) {
            mock_seeded = false;
        }

        // Acquire on the capture cadence: mock values or the real sensor.
        if (act.capture) {
            if (live.mock_enabled) {
                savia_reading_t r10 = { .ts_ms = now_ms, .port = 1, .depth_cm = 10,
                                        .kind = READING_SOIL_MOISTURE, .value = 0.70f };
                savia_reading_t r30 = { .ts_ms = now_ms, .port = 1, .depth_cm = 30,
                                        .kind = READING_SOIL_MOISTURE, .value = 0.74f };
                savia_reading_t rta = { .ts_ms = now_ms, .port = 1, .depth_cm = 0,
                                        .kind = READING_AIR_TEMPERATURE, .value = 22.0f };
                storage_append_reading(&r10);
                storage_append_reading(&r30);
                storage_append_reading(&rta);
            } else {
                for (uint8_t i = 0; i < live.sensor_count; i++) {
                    savia_reading_t buf[8];
                    int n = sensor_measure(&live.sensors[i], buf, 8);
                    for (int k = 0; k < n; k++) {
                        buf[k].ts_ms = now_ms;
                        storage_append_reading(&buf[k]);
                    }
                }
            }
        }

        if (live.lora_enabled) {
            lora_cycle();
        }

        // 2/3. Daily cycle: on RP2350 run the on-device LSTM; on the Pico WH the
        //      app runs inference from the served forecast, so here we just mark it.
        if (act.daily) {
            LOG_INFO("sched: daily cycle (hour=%u)\n", (unsigned) live.daily_hour);
            if (inference_on_device()) {
                // TODO(ml): run the LSTM here on RP2350.
            }
        }

        ble_poll(/*budget_ms=*/5000);
        // Persist if the app changed config: re-snapshot under the lock so the
        // flash write sees a non-torn copy of the BLE-owned struct.
        if (ble_take_config_dirty()) {
            cfg_lock(); live = cfg; cfg_unlock();
            config_store_save(&live);
        }

        // 4. Nap until the next mandatory wake (capped by sleep_s), or the button.
        //    With deep sleep enabled we power the radio down first (real low power,
        //    not discoverable until the button); disabled (default) we stay awake.
        uint32_t nap = timed
            ? scheduler_next_sleep_s(&sched, now_ms, live.sleep_seconds,
                                     live.capture_interval_s, live.daily_hour)
            : live.sleep_seconds;
        savia_wake_reason_t why;
        if (live.deep_sleep_enabled) {
            ble_radio_suspend();
            why = power_deep_sleep(&live, nap);
            ble_radio_resume();
        } else {
            why = power_deep_sleep(&live, nap);
        }
        if (why == SAVIA_WAKE_BUTTON) {
            // Technician pressed the button: open a longer BLE service window.
            ble_poll(/*budget_ms=*/30000);
            if (ble_take_config_dirty()) {
                cfg_lock(); live = cfg; cfg_unlock();
                config_store_save(&live);
            }
        }

        // On-demand LoRa ping from the app (the nap wakes early to service it):
        // (re)open the module on the configured pins, join, send one confirmed
        // uplink, capture the ACK's RSSI/SNR, and persist the signal.
        if (ble_take_lora_ping()) {
            uint64_t pnow = clock_is_set()
                ? clock_now(to_ms_since_boot(get_absolute_time())) : 0;
            lora_ping(live.lora_uart_tx_gpio, live.lora_uart_rx_gpio, pnow);
            lora_persist_signal(&cfg);
        }

        // Raw AT command from the app's terminal: run it on the module, result is
        // read back by the app over the "at" data_request.
        char atcmd[SAVIA_AT_CMD_MAX];
        if (ble_take_lora_at(atcmd, sizeof atcmd)) {
            lora_at(live.lora_uart_tx_gpio, live.lora_uart_rx_gpio, atcmd);
        }
    }
}
