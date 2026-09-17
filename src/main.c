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
#include "savia/ble_lock.h"
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
#include "savia/mock_soil.h"
#include "savia/weather.h"
#include "savia/scheduler.h"
#include "savia/sensor_catalog.h"
#include "savia/storage_store.h"
#include "savia/log.h"
#include "savia/uptime.h"
#include "savia/wdt.h"

// Serialize cfg access against the BLE write path. In threadsafe-background mode
// BTstack (incl. handle_config_write's `*g_cfg = next`) runs under the cyw43
// async_context lock; holding it here makes the ~196 B cfg copy atomic vs the
// supervisor, preventing torn reads of cfg.sensors[i]. No-op when BLE is off.
// The same lock guards the readings ring: ingest/clear/mock run in BLE context.
static inline void cfg_lock(void)   { savia_ble_lock(); }
static inline void cfg_unlock(void) { savia_ble_unlock(); }

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
        int n = sensor_measure(&cfg->sensors[i], buf, 8);   // slow: outside the lock
        cfg_lock();
        for (int k = 0; k < n; k++) {
            buf[k].ts_ms = now_ms;
            buf[k].port = (uint8_t)(i + 1);   // slot index -> logical port
            storage_append_reading(&buf[k]);
        }
        cfg_unlock();
    }
}

// Mock mode: top up the replayed soil window around the current hour. Needs the
// wall clock, so it is a no-op until the first sync.
static void mock_top_up(const station_config_t *cfg) {
    if (!clock_is_set()) return;
    uint64_t now = clock_now(savia_uptime_ms());
    cfg_lock();
    size_t n = mock_soil_fill(now, cfg->utc_offset_min);
    cfg_unlock();
    if (n > 2) LOG_INFO("mock: %u replayed soil readings added\n", (unsigned) n);
    else if (n) LOG_DEBUG("mock: %u replayed soil readings added\n", (unsigned) n);
}

// After a deep-sleep wake the scheduler is a fresh struct: re-derive each input's
// next due from its newest stored reading so the cadence continues.
static void seed_schedule_from_storage(savia_scheduler_t *s, const station_config_t *cfg) {
    size_t n = storage_reading_count();
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
        if (!savia_slot_used(&cfg->sensors[i]) ||
            sensor_type_is_output(cfg->sensors[i].type)) continue;
        uint64_t newest = 0;
        for (size_t k = 0; k < n; k++) {
            const savia_reading_t *r = storage_reading_at(k);
            if (r && r->port == (uint8_t)(i + 1) && r->ts_ms > newest) newest = r->ts_ms;
        }
        scheduler_seed_sensor(s, i, newest, cfg);
    }
}

// Time source for log line stamps: wall-clock once synced, uptime before that.
static uint64_t log_clock(bool *wall) {
    uint64_t up = savia_uptime_ms();
    if (clock_is_set()) { *wall = true; return clock_now(up); }
    *wall = false;
    return up;
}

static void log_flush(void) { stdio_flush(); }

int main(void) {
    stdio_init_all();
    // A wake from deep sleep is a reboot: read the context the previous power
    // cycle left in the always-on scratch registers before anything else runs.
    savia_deep_wake_t wake;
    power_deep_wake_info(&wake);
    savia_wdt_start();
    // Cold boot: let the USB-CDC host attach so EARLY boot logs are visible.
    // After a deep sleep every second awake is battery: only a short settle.
    sleep_ms(wake.resumed ? 300 : 2500);
    savia_log_set_clock(log_clock);   // timestamp every log line
    savia_log_set_flush(log_flush);  // drain USB-CDC per line: no dropped logs
    savia_wdt_feed();
    if (savia_wdt_caused_reboot())
        LOG_WARN("boot: reset by the watchdog (the firmware stopped responding)\n");

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
#if SAVIA_MOCK_DATA
    // A MOCK image always replays. Without a saved "mock on" the ring may hold a
    // real image's readings: it is wiped once loaded (below).
    bool drop_real = !cfg.mock_enabled || config_store_is_factory();
    cfg.mock_enabled = true;
#else
    const bool drop_real = false;
#endif
    savia_log_set_level(cfg.log_level);

    // Seed the sync ring from flash: the pre-outage reference. Does NOT set the
    // clock (time passed while off) -- the first live sync measures the outage.
    if (clock_store_load())
        printf("clock: sync ring restored (last known %llu ms)\n",
               (unsigned long long) clock_last_known());
    // Deep-sleep wake: the always-on timer kept counting while the core was off,
    // so the clock continues from it (no outage, no provisional readings).
    if (wake.resumed) {
        uint64_t up_w = savia_uptime_ms();
        if (clock_resume_from_aon(wake.now_wall_ms, up_w, wake.clock_uncertainty_ms)) {
            LOG_INFO("power: woke from deep sleep after %u s (%s); clock from the AON timer, +/-%u ms\n",
                     (unsigned)(wake.slept_ms / 1000u), wake.button ? "button" : "timer",
                     (unsigned) wake.clock_uncertainty_ms);
        } else {
            LOG_WARN("power: deep-sleep clock implausible -> cold start\n");
            wake.resumed = false;
        }
    }

    power_init(&cfg);

    // Recovery: power on with the wake button held -> factory reset (wipes the
    // password and all settings back to defaults). The reverted BLE name ("Savia")
    // is the visible confirmation. Cold boots only: the same button pressed
    // during a deep sleep is a service request, not a reset.
    if (!wake.resumed && power_reset_button_held(&cfg, SAVIA_FACTORY_RESET_HOLD_MS)) {
        LOG_WARN("button held at boot -> factory reset (clearing password + config)\n");
        config_load_defaults(&cfg);
        config_store_erase();
        savia_log_set_level(cfg.log_level);
    }

    sensor_init(&cfg);
    // Outputs are OFF at boot by contract (actuator.h): drive the pins to match
    // before anything else can connect and ask for a state.
    out_slot_t out_slots[SAVIA_MAX_SENSORS] = { 0 };
    sync_output_pins(out_slots, cfg.sensors);
    storage_init();
    // Bring back the readings the last power cycle had.
    storage_store_load();
    if (drop_real) {
        storage_clear();
        if (storage_take_dirty()) storage_store_save();   // the wipe lands before the flag
        if (!config_store_is_factory()) config_store_save(&cfg);
    }
    // Mock mode replays the dataset instead of the probes (filled once the clock is known).
    bool mock_active = cfg.mock_enabled;
    if (mock_active) LOG_INFO("mock: soil comes from the dataset replay\n");
    savia_wdt_feed();
    ble_init(&cfg);
    savia_wdt_feed();
    status_led_init();   // onboard LED: solid=paired, 1s blink=BLE on, 3s blink=operating
    if (cfg.lora_enabled) {
        lora_init(&cfg);
    }
    // Show the last LoRa signal (from a prior power cycle) until a fresh ping.
    lora_seed_last_signal(cfg.lora_last_rssi_dbm, cfg.lora_last_snr_ddb,
                          cfg.lora_last_signal_ms);
    // Deep-sleep wake: same LoRa session as before the nap (the module stayed
    // powered and joined): no BOOT frame, coords already sent, period gate kept.
    if (wake.resumed && cfg.lora_enabled) {
        uint64_t up_l = savia_uptime_ms();
        lora_restore_cycle_state(wake.lora_last_attempt_s, wake.lora_last_soil_hour_s,
                                 clock_now(up_l), &cfg);
    }

    // First thing after bring-up: one LoRa cycle. On a cold boot it ALWAYS runs:
    // its downlink carries the clock (fresh time even when the LKG ring already
    // seeded one) and flushes any downlinks queued while the station was off.
    // After a deep sleep it only runs when the period is due. Then, if the clock
    // is still not set, hold a short BLE window for a phone that connects first.
    if (cfg.lora_enabled) lora_cycle(&cfg);
    if (!clock_is_set()) {
        ble_poll(/*budget_ms=*/3000);
    }
    // Woken by the technician's button: the same service window the light nap
    // offers, so the phone can connect before the next power-off.
    if (wake.resumed && wake.button) {
        LOG_INFO("power: button wake -> BLE service window\n");
        ble_poll(/*budget_ms=*/30000);
        if (ble_take_config_dirty()) {
            cfg_lock(); station_config_t snap = cfg; cfg_unlock();
            config_store_save(&snap);
        }
    }
    clock_persist_if_dirty();
    // Now that the clock is (usually) known, decide whether the stored TA window
    // still describes the hour we are in. A downlink during the cycle above wins.
    {
        uint64_t up0 = savia_uptime_ms();
        storage_store_adopt_weather(clock_is_set() ? clock_now(up0) : 0);
    }

    printf("savia_c up: on_device_inference=%d, sensors=%u, sleep=%us, capture=%us, daily=%02u:%02u\n",
           inference_on_device(), config_sensor_count(&cfg), cfg.sleep_seconds,
           cfg.capture_interval_s, cfg.daily_hour, cfg.daily_min);

    // Dev self-test under mock: one LSTM run over the replay's embedded window, so
    // the logs show that TFLM runs on this board and how far it lands from the host.
    if (inference_on_device() && mock_active && !wake.resumed) inference_selftest();

    savia_scheduler_t sched;
    scheduler_init(&sched);
    // Deep-sleep wake: continue the plan instead of sampling everything at once.
    if (wake.resumed) {
        sched.last_daily_day = wake.last_daily_day;
        seed_schedule_from_storage(&sched, &cfg);
    }

    bool was_timed = clock_is_set();   // back-fill trigger: unset -> set transition

    for (;;) {
        savia_wdt_feed();
#if SAVIA_WDT_SELFTEST
        // Bench check (make flash WDT_SELFTEST=ON): hang once, the watchdog must reset us.
        if (!savia_wdt_caused_reboot()) {
            LOG_WARN("wdt selftest: hanging on purpose, expect a reset in %u ms\n", SAVIA_WDT_TIMEOUT_MS);
            for (;;) tight_loop_contents();
        }
#endif
        uint64_t up = savia_uptime_ms();
        bool timed = clock_is_set();
        uint64_t now_ms = timed ? clock_now(up) : up;

        // First sync this power cycle: rebase provisional (uptime-stamped) readings
        // to wall time. delta = epoch - uptime, constant for the whole power cycle.
        if (timed && !was_timed) {
            cfg_lock();
            size_t fixed = storage_rebase_provisional(clock_now(up) - up);
            cfg_unlock();
            if (fixed) LOG_INFO("storage: back-filled %u provisional readings\n",
                                (unsigned) fixed);
            was_timed = true;
        }

        // A sync moved the clock back (a poisoned time repaired): readings stamped
        // in that future move back with it, and LoRa resends the hours it sent.
        if (timed) {
            uint64_t back_ms = 0;
            size_t fixed = 0;
            cfg_lock();
            bool stepped = clock_take_step_back(&back_ms);
            // Under mock the replay is rebuilt for the corrected clock (below) instead.
            if (stepped && mock_active) storage_clear();
            else if (stepped) fixed = storage_rewind_future(clock_now(savia_uptime_ms()), back_ms);
            cfg_unlock();
            if (stepped) {
                lora_forget_future_soil(clock_now(savia_uptime_ms()));
                LOG_WARN("clock: moved back %llu s; %u future readings fixed\n",
                         (unsigned long long) (back_ms / 1000u), (unsigned) fixed);
                if (mock_active) LOG_INFO("mock: replay rebuilt for the corrected clock\n");
            }
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

        // The app switched mock on or off: start from an empty ring either way, so
        // replayed and measured soil never share a window.
        if (live.mock_enabled != mock_active) {
            cfg_lock();
            storage_clear();
            cfg_unlock();
            mock_active = live.mock_enabled;
            LOG_INFO("mock: %s, stored readings cleared\n", mock_active ? "on" : "off");
        }

        // Acquire: the replay stands in for every probe under mock; otherwise only
        // the sensors whose own cadence elapsed (act.capture_mask bit i == sensor i).
        if (mock_active) {
            mock_top_up(&live);
        } else if (act.capture_mask) {
            capture_slots(&live, act.capture_mask, now_ms);
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
        if (mock_active && !timed) mock_top_up(&live);   // the downlink may have set the clock

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
            // Real deep sleep (RP2350 power manager, P1.7): the chip powers off
            // and reboots on the alarm or the button; main() continues from the
            // context stashed in the always-on registers. Only for a nap long
            // enough to pay for the reboot, with the clock known (the AON timer
            // carries it) and nothing pending from the app; otherwise the light
            // nap. If the power manager refuses, the light nap is the fallback
            // for the rest of this power cycle.
            uint64_t up_n = savia_uptime_ms();
            uint64_t wall_n = clock_is_set() ? clock_now(up_n) : 0;
            bool app_busy = ble_is_connected() || ble_lora_ping_pending() ||
                            ble_lora_at_pending() || ble_sdi12_pending() ||
                            ble_act_pending() || ble_infer_pending() ||
                            ble_config_dirty_pending();
            // A held LoRa time lives in RAM: a power-off would forget it.
            bool clock_hold = clock_repair_pending();
            ble_radio_suspend();
            if (wall_n && nap >= SAVIA_DEEP_SLEEP_MIN_S && !app_busy && !clock_hold &&
                power_deep_sleep_available()) {
                savia_deep_sleep_ctx_t ctx = {
                    .now_wall_ms = wall_n,
                    .uptime_ms = up_n,
                    .nap_s = nap,
                    .lora_last_attempt_s = lora_last_attempt_epoch_s(wall_n),
                    .lora_last_soil_hour_s = lora_last_soil_hour_s(),
                    .last_daily_day = sched.last_daily_day,
                    .clock_uncertainty_ms = clock_uncertainty_ms(),
                };
                power_deep_sleep_off(&live, &ctx);   // returns only when refused
            }
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
                uint64_t up2 = savia_uptime_ms();
                uint64_t inow = clock_is_set() ? clock_now(up2) : up2;
                LOG_INFO("BLE: running on-demand inference\n");
                // Sample first: the app can ask at any minute, and the model's
                // newest step has to be a real reading of THIS hour, not a copy
                // carried over from the last scheduled capture.
                if (mock_active) mock_top_up(&live);
                else capture_slots(&live, 0xFF, inow);
                inference_run_daily(inow);
            }
        }

        // On-demand LoRa ping from the app (the nap wakes early to service it):
        // (re)open the module on the configured pins, join, send one confirmed
        // uplink, capture the ACK's RSSI/SNR, and persist the signal.
        if (ble_take_lora_ping()) {
            uint64_t pnow = clock_is_set()
                ? clock_now(savia_uptime_ms()) : 0;
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
