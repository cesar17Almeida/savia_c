// savia_c entry point + supervisor loop.
//
// The station's four responsibilities (same as savia_py): acquire -> aggregate
// (in storage) -> infer -> serve (BLE). On a microcontroller we wrap them in a
// deep-sleep cycle: wake, sample, do periodic work, sleep until the next cycle
// or the button.
#include <stdio.h>
#include <string.h>
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
#include "savia/status_led.h"
#include "savia/lora.h"
#include "savia/lora_codec.h"   // lora_apply_config_tlv + LORA_DOWNLINK_MAX
#include "savia/sdi12.h"        // probe console (op "sdi12")
#include "savia/actuator.h"     // actuator slot state (op "act")
#include "savia/inference.h"
#include "savia/weather.h"
#include "savia/scheduler.h"
#include "savia/storage_store.h"
#include "savia/log.h"

// Mock dataset baseline (~20 Jun 2026 UTC). Shared by the mock seeding and the
// boot inference self-test so both anchor to the same window.
#define MOCK_BASE_MS 1782000000000ULL

// Dev/mock: seed 48 h of hourly readings (HS10, HS30) at boot -- the LSTM's past
// window -- so the app has a dataset before live sampling accumulates. Also seeds
// a mock TA forecast into the weather cache (past 48 h + next 24 h) so the
// on-device LSTM path can build a complete window under mock: HS10/HS30 come from
// these readings, TA (past + future) from the weather cache.
//
// NO air-temperature READING is seeded, deliberately. A station whose only probe
// is a buried soil sensor has no air thermometer, and readings are attributed by
// PORT: a mock TA on port 1 shows up in the soil probe's own history as if the
// probe had measured the air, which it cannot. Air temperature reaches this
// station as a forecast, so the weather cache below is its only honest home.
static void seed_mock_readings(void) {
    const uint64_t base = MOCK_BASE_MS;   // ~20 Jun 2026 UTC (mock baseline), hour-aligned
    // h = 47..0, NOT 48..1: the newest sample has to land in the base hour itself.
    // Seeded one hour short, the window's last step would be a LOCF copy and
    // lstm_gather_inputs would refuse it as stale (LSTM_MAX_STALE_HOURS = 0), which
    // is exactly what the boot self-test needs NOT to hit.
    for (int h = 47; h >= 0; h--) {
        uint64_t ts = base - (uint64_t) h * 3600000ULL;
        float drift = (float) h * 0.002f;       // older = slightly wetter (dry-down)
        savia_reading_t r10 = { .ts_ms = ts, .port = 1, .depth_cm = 10,
                                .kind = READING_SOIL_MOISTURE, .value = 0.70f + drift };
        savia_reading_t r30 = { .ts_ms = ts, .port = 1, .depth_cm = 30,
                                .kind = READING_SOIL_MOISTURE, .value = 0.74f + drift };
        storage_append_reading(&r10);
        storage_append_reading(&r30);
    }
    // Mock TA forecast: past[i] ends at the latest hour, future is the next 24 h.
    float ta_past[WEATHER_PAST_MAX], ta_future[WEATHER_FUTURE_MAX];
    for (int i = 0; i < WEATHER_PAST_MAX; i++)
        ta_past[i] = 20.0f + (float) i * 0.05f;      // oldest -> newest
    for (int i = 0; i < WEATHER_FUTURE_MAX; i++)
        ta_future[i] = 22.0f + (float) i * 0.10f;
    weather_set(ta_past, WEATHER_PAST_MAX, ta_future, WEATHER_FUTURE_MAX, base);
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

// Persist the clock sync ring if a sync advanced it. Snapshots the ring under the
// BLE lock (a time_sync write runs in the cyw43 context) so the flash write can't
// race it -- same discipline as the config save.
static void clock_persist_if_dirty(void) {
    uint8_t blob[CLOCK_RING_BLOB_MAX];
    size_t n = 0;
    bool dirty;
    cfg_lock();
    dirty = clock_take_ring_dirty();
    if (dirty) n = clock_serialize_ring(blob, sizeof blob);
    cfg_unlock();
    if (dirty) clock_store_save(blob, n);
}

// What sync_output_pins remembers between cycles: enough to release a pin that
// stops being an output, and no more.
typedef struct { savia_sensor_type_t type; uint8_t gpio; } out_slot_t;

// Drive every output slot to its logical state, and release the ones that stopped
// being outputs. Until a pin is driven it floats, so a relay board -- not us --
// would decide the valve's state at boot; and an actuator deleted from the app
// while ON would otherwise stay energised with nothing left to switch it off.
// Cheap and self-diffing: call it at boot and once per cycle. `seen` is the
// previous table and is updated in place; an unchanged slot is left alone so a
// live output never glitches.
static void sync_output_pins(out_slot_t *seen, const savia_sensor_slot_t *now) {
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
        uint8_t port = (uint8_t)(i + 1);
        bool was = seen[i].type == SENSOR_ACTUATOR_DIGITAL;
        bool is  = now[i].type == SENSOR_ACTUATOR_DIGITAL;
        bool moved = now[i].gpio != seen[i].gpio;
        if (was && (!is || moved)) {              // dropped, retyped, or moved pin
            gpio_init(seen[i].gpio);
            gpio_set_dir(seen[i].gpio, GPIO_OUT);
            gpio_put(seen[i].gpio, 0);
            actuator_set(port, false);
            LOG_INFO("act: port %u released (GP%u -> OFF)\n", port, seen[i].gpio);
        }
        if (is && (!was || moved)) {              // new output: drive it, don't float
            gpio_init(now[i].gpio);
            gpio_set_dir(now[i].gpio, GPIO_OUT);
            gpio_put(now[i].gpio, actuator_is_on(port));
        }
        seen[i].type = now[i].type;
        seen[i].gpio = now[i].gpio;
    }
}

// Sample the slots flagged in `mask` and store what they return, stamped `now_ms`.
// Shared by the scheduled capture and the on-demand inference, which samples first
// so it never forecasts from a window whose newest step is a copy.
static void capture_slots(const station_config_t *cfg, uint8_t mask, uint64_t now_ms) {
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
        if (!savia_slot_used(&cfg->sensors[i])) continue;   // free slot
        if (!(mask & (1u << i))) continue;                  // not due this tick
        savia_reading_t buf[8];
        int n = sensor_measure(&cfg->sensors[i], buf, 8);
        for (int k = 0; k < n; k++) {
            buf[k].ts_ms = now_ms;
            buf[k].port = (uint8_t)(i + 1);   // slot index -> logical port
            storage_append_reading(&buf[k]);
        }
    }
}

// Time source for log line stamps: wall-clock once synced, uptime before that.
static uint64_t log_clock(bool *wall) {
    uint64_t up = to_ms_since_boot(get_absolute_time());
    if (clock_is_set()) { *wall = true; return clock_now(up); }
    *wall = false;
    return up;
}

static void log_flush(void) { stdio_flush(); }

int main(void) {
    stdio_init_all();
    sleep_ms(2500);   // let the USB-CDC host attach so EARLY boot logs are visible
    savia_log_set_clock(log_clock);   // timestamp every log line
    savia_log_set_flush(log_flush);  // drain USB-CDC per line: no dropped logs

    station_config_t cfg;
    config_load_defaults(&cfg);
    if (config_store_load(&cfg)) {
        LOG_INFO("config: restored from flash (sleep=%us capture=%us lora=%us)\n",
                 cfg.sleep_seconds, cfg.capture_interval_s, cfg.lora_period_s);
    } else {
        LOG_WARN("config: no valid record -> factory defaults\n");
    }
    // A record written on a board that carries the model can land on one that
    // doesn't (same layout): don't advertise LOCAL where nothing can run it.
    if (cfg.inference_mode == SAVIA_INFER_LOCAL && !inference_on_device()) {
        cfg.inference_mode = SAVIA_INFER_FORWARD;
        LOG_WARN("config: LOCAL inference unavailable in this build -> FORWARD\n");
    }
    savia_log_set_level(cfg.log_level);

    // Seed the sync ring from flash: the pre-outage reference. Does NOT set the
    // clock (time passed while off) -- the first live sync measures the outage.
    if (clock_store_load())
        printf("clock: sync ring restored (last known %llu ms)\n",
               (unsigned long long) clock_last_known());

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
    // Outputs are OFF at boot by contract (actuator.h): drive the pins to match
    // before anything else can connect and ask for a state.
    out_slot_t out_slots[SAVIA_MAX_SENSORS] = { 0 };
    sync_output_pins(out_slots, cfg.sensors);
    storage_init();
    // Bring back the readings the last power cycle had. Before the mock seed on
    // purpose: a dev board still gets its synthetic window appended on top.
    storage_store_load();
    bool mock_seeded = false;
    if (cfg.mock_enabled) { seed_mock_readings(); mock_seeded = true; }   // dev dataset
    ble_init(&cfg);
    status_led_init();   // onboard LED: solid=paired, 1s blink=BLE on, 3s blink=operating
    if (cfg.lora_enabled) {
        lora_init(&cfg);
    }
    // Show the last LoRa signal (from a prior power cycle) until a fresh ping.
    lora_seed_last_signal(cfg.lora_last_rssi_dbm, cfg.lora_last_snr_ddb,
                          cfg.lora_last_signal_ms);

    // First thing after bring-up: one LoRa cycle, ALWAYS. Its downlink carries the
    // clock (fresh time even when the LKG ring already seeded one) and flushes any
    // downlinks queued while the station was off. Then, if the clock is still not
    // set, hold a short BLE window for a phone that connects first.
    if (cfg.lora_enabled) lora_cycle(&cfg);
    if (!clock_is_set()) {
        ble_poll(/*budget_ms=*/3000);
    }
    clock_persist_if_dirty();
    // Now that the clock is (usually) known, decide whether the stored TA window
    // still describes the hour we are in. A downlink during the cycle above wins.
    {
        uint64_t up0 = to_ms_since_boot(get_absolute_time());
        storage_store_adopt_weather(clock_is_set() ? clock_now(up0) : 0);
    }

    printf("savia_c up: on_device_inference=%d, sensors=%u, sleep=%us, capture=%us, daily=%02u:%02u\n",
           inference_on_device(), config_sensor_count(&cfg), cfg.sleep_seconds,
           cfg.capture_interval_s, cfg.daily_hour, cfg.daily_min);

    // On-device inference self-test (dev, mock only): run one LSTM inference over the
    // seeded 48 h mock window right at boot so the logs show whether TFLM allocates,
    // Invoke() runs, and how big the arena really is on this board. Independent of
    // clock/BLE -- the mock data is anchored at MOCK_BASE_MS. Logs go to serial and
    // the BLE "logs" channel. inference_run_daily is a no-op off-device.
    if (inference_on_device() && cfg.mock_enabled) {
        LOG_INFO("selftest: running on-device LSTM over the mock window...\n");
        int rc = inference_run_daily(MOCK_BASE_MS);
        LOG_INFO("selftest: inference_run_daily rc=%d\n", rc);
    }

    savia_scheduler_t sched;
    scheduler_init(&sched);

    bool was_timed = clock_is_set();   // back-fill trigger: unset -> set transition

    for (;;) {
        uint64_t up = to_ms_since_boot(get_absolute_time());
        bool timed = clock_is_set();
        uint64_t now_ms = timed ? clock_now(up) : up;

        // First sync this power cycle: rebase provisional (uptime-stamped) readings
        // to wall time. delta = epoch - uptime, constant for the whole power cycle.
        if (timed && !was_timed) {
            size_t fixed = storage_rebase_provisional(clock_now(up) - up);
            if (fixed) LOG_INFO("storage: back-filled %u provisional readings\n",
                                (unsigned) fixed);
            was_timed = true;
        }

        // Take a consistent snapshot of the BLE-owned cfg so the rest of the
        // iteration (incl. the long SDI-12 read of cfg.sensors[i]) sees a stable
        // copy even if a BLE config write lands mid-cycle.
        station_config_t live;
        cfg_lock();
        live = cfg;
        cfg_unlock();

        // A config write may have added, moved or deleted an actuator: no output
        // is left floating or energised without a slot behind it.
        sync_output_pins(out_slots, live.sensors);

        // 1. Decide what's due now. The sleep time is only the low-power tick;
        //    the schedule forces the mandatory wakes, per sensor (each on its own
        //    cadence). Before time is set we capture every sensor each cycle so we
        //    never sit idle without data (all mask bits set).
        savia_sched_action_t act = timed
            ? scheduler_tick(&sched, now_ms, &live)
            : (savia_sched_action_t){ .capture_mask = 0xFF, .daily = false };

        // Sync mock state if the app toggled it (re-seed the dataset on enable).
        if (live.mock_enabled && !mock_seeded) {
            storage_clear(); seed_mock_readings(); mock_seeded = true;
        } else if (!live.mock_enabled && mock_seeded) {
            mock_seeded = false;
        }

        // Acquire whatever is due now: mock values, or only the real sensors whose
        // own cadence elapsed this tick (act.capture_mask bit i == sensor i).
        if (act.capture_mask) {
            if (live.mock_enabled) {
                savia_reading_t r10 = { .ts_ms = now_ms, .port = 1, .depth_cm = 10,
                                        .kind = READING_SOIL_MOISTURE, .value = 0.70f };
                savia_reading_t r30 = { .ts_ms = now_ms, .port = 1, .depth_cm = 30,
                                        .kind = READING_SOIL_MOISTURE, .value = 0.74f };
                // Soil only, for the reason in seed_mock_readings: a mock air
                // temperature on port 1 would masquerade as a reading of the probe.
                storage_append_reading(&r10);
                storage_append_reading(&r30);
            } else {
                capture_slots(&live, act.capture_mask, now_ms);
            }
        }

        if (!live.lora_enabled) {
            LOG_INFO("LoRa: disabled in cfg, cycle skipped\n");
        }
        if (live.lora_enabled) {
            lora_cycle(&live);
            // A CONFIG downlink arrived: apply the TLV to the BLE-owned cfg with
            // the same clamps as the BLE patch, persist, and queue the CFG_ACK.
            uint8_t tlv[LORA_DOWNLINK_MAX];
            size_t tlv_len;
            if (lora_take_config_tlv(tlv, sizeof tlv, &tlv_len)) {
                uint8_t ok = 0, bad = 0;
                cfg_lock();
                lora_apply_config_tlv(tlv, tlv_len, &cfg, &ok, &bad);
                if (cfg.inference_mode == SAVIA_INFER_LOCAL && !inference_on_device()) {
                    cfg.inference_mode = SAVIA_INFER_FORWARD;   // LOCAL impossible here
                    if (ok) ok--;
                    bad++;
                }
                live = cfg;
                cfg_unlock();
                config_store_save(&live);
                lora_set_cfg_ack(ok, bad);
                LOG_INFO("LoRa config patch: %u applied, %u rejected\n", ok, bad);
            }
        }

        // 2/3. Daily cycle at daily_hour LOCAL: run the LSTM only in LOCAL mode on
        //      an on-device build; in FORWARD the data is served/uplinked instead.
        if (act.daily) {
            LOG_INFO("sched: daily cycle (local %02u:%02u, mode=%s)\n",
                     (unsigned) live.daily_hour, (unsigned) live.daily_min,
                     live.inference_mode == SAVIA_INFER_LOCAL ? "local" : "forward");
            if (live.inference_mode == SAVIA_INFER_LOCAL && inference_on_device()) {
                // Verified on the Pico 2 W (2026-07-04): arena 162 KB, Invoke 269 ms.
                inference_run_daily(now_ms);
            }
        }

        ble_poll(/*budget_ms=*/5000);
        // Persist if the app changed config: re-snapshot under the lock so the
        // flash write sees a non-torn copy of the BLE-owned struct.
        if (ble_take_config_dirty()) {
            cfg_lock(); live = cfg; cfg_unlock();
            config_store_save(&live);
        }
        // Persist the sync ring if this cycle's LoRa downlink or a BLE time_sync
        // advanced it (keeps the pre-outage reference fresh for the next reboot).
        clock_persist_if_dirty();

        // 3b. Persist the measurement state, if anything changed. Here and not
        //     elsewhere: BLE has just been serviced and the nap is next, so the
        //     ~150 ms of paused radio this costs lands where nothing is waiting on
        //     it. A phone mid-transfer would notice it anywhere earlier.
        if (storage_take_dirty()) storage_store_save();

        // 4. Nap until the next mandatory wake (capped by sleep_s), or the button.
        //    With deep sleep enabled we power the radio down first (real low power,
        //    not discoverable until the button); disabled (default) we stay awake.
        uint32_t nap = timed
            ? scheduler_next_sleep_s(&sched, now_ms, &live)
            : live.sleep_seconds;
        // The scheduler doesn't know the radio: also wake for the LoRa cadence.
        if (live.lora_enabled) {
            uint32_t lora_due = lora_secs_until_due(&live);
            if (lora_due < nap) nap = lora_due ? lora_due : 1;
        }
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

        if (ble_take_infer_trigger()) {
            if (live.inference_mode == SAVIA_INFER_LOCAL && inference_on_device()) {
                // Recompute the wall clock: the nap above may have advanced it
                // well past the now_ms captured at the top of the loop.
                uint64_t up2 = to_ms_since_boot(get_absolute_time());
                uint64_t inow = clock_is_set() ? clock_now(up2) : up2;
                LOG_INFO("BLE: running on-demand inference\n");
                // Sample first: the app can ask at any minute, and the model's
                // newest step has to be a real reading of THIS hour, not a copy
                // carried over from the last scheduled capture.
                if (!live.mock_enabled) capture_slots(&live, 0xFF, inow);
                inference_run_daily(inow);
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
            clock_persist_if_dirty();   // the ping's downlink may carry a fresh clock
        }

        // Raw AT command from the app's terminal: run it on the module, result is
        // read back by the app over the "at" data_request.
        char atcmd[SAVIA_AT_CMD_MAX];
        if (ble_take_lora_at(atcmd, sizeof atcmd)) {
            lora_at(live.lora_uart_tx_gpio, live.lora_uart_rx_gpio, atcmd);
        }

        // Raw SDI-12 probe command from the app's sensor console (blocking,
        // ~1.5 s); the app reads the result back via the "sdi12" data_request.
        char sdicmd[SDI12_CMD_MAX];
        uint8_t sdigpio;
        if (ble_take_sdi12(sdicmd, sizeof sdicmd, &sdigpio)) {
            sdi12_console_run(sdigpio, sdicmd);
        }

        // Actuator switch from the app: only a configured ACTUATOR slot drives
        // its GPIO. State is RAM-only on purpose -- everything is OFF after boot.
        uint8_t aport; bool aon;
        if (ble_take_act(&aport, &aon)) {
            if (aport >= 1 && aport <= SAVIA_MAX_SENSORS &&
                live.sensors[aport - 1].type == SENSOR_ACTUATOR_DIGITAL) {
                uint8_t g = live.sensors[aport - 1].gpio;
                gpio_set_dir(g, GPIO_OUT);   // already an output (sync_output_pins)
                gpio_put(g, aon);
                actuator_set(aport, aon);
                LOG_INFO("act: port %u (GP%u) -> %s\n", aport, g, aon ? "ON" : "OFF");
            } else {
                LOG_WARN("act: port %u is not an actuator slot\n", aport);
            }
        }
    }
}
