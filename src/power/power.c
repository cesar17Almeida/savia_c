#include "savia/power.h"
#include "savia/ble.h"
#include "savia/log.h"
#include "savia/uptime.h"
#include "pico/stdlib.h"
#include <string.h>

#if PICO_RP2350
#include "hardware/powman.h"
#include "hardware/structs/powman.h"
#include "hardware/sync.h"      // __wfi
#include "pico/stdio.h"
#endif

// The button is wired to GND with an internal pull-up, so it reads low when
// pressed (active-low).
void power_init(const station_config_t *cfg) {
    gpio_init(cfg->wake_button_gpio);
    gpio_set_dir(cfg->wake_button_gpio, GPIO_IN);
    gpio_pull_up(cfg->wake_button_gpio);
}

bool power_reset_button_held(const station_config_t *cfg, uint32_t hold_ms) {
    const uint32_t step_ms = 50;
    uint32_t elapsed = 0;
    while (elapsed < hold_ms) {
        if (gpio_get(cfg->wake_button_gpio)) return false;  // released early -> abort
        sleep_ms(step_ms);
        elapsed += step_ms;
    }
    return true;   // held low the whole window
}

// Light nap: sleep_ms idles the core (WFE) between polls of the button and of the
// app's requests. The radio is already down when deep sleep is enabled.
savia_wake_reason_t power_deep_sleep(const station_config_t *cfg, uint32_t seconds) {
    const uint32_t step_ms = 50;
    uint32_t elapsed = 0;
    while (elapsed < seconds * 1000u) {
        if (!gpio_get(cfg->wake_button_gpio)) {  // active-low press
            return SAVIA_WAKE_BUTTON;
        }
        // Any app request the supervisor must service ends the nap early: LoRa
        // ping / AT terminal / SDI-12 console / actuator switch / inference.
        if (ble_lora_ping_pending() || ble_lora_at_pending() ||
            ble_sdi12_pending() || ble_act_pending() || ble_infer_pending() ||
            ble_config_dirty_pending()) {   // persist config writes promptly
            return SAVIA_WAKE_TIMER;
        }
        sleep_ms(step_ms);
        elapsed += step_ms;
    }
    return SAVIA_WAKE_TIMER;
}

// --- real deep sleep: RP2350 power manager (P1.7) ------------------------------
#if PICO_RP2350

// Scratch layout (always-on domain: survives the power-off, cleared by a reset):
//   [0] magic               [1..2] wall clock ms at power-off (lo, hi)
//   [3] planned nap s       [4] LoRa last attempt (epoch s)
//   [5] last daily day      [6] clock uncertainty ms carried into the sleep
//   [7] newest uplinked soil hour (epoch s)
#define DS_MAGIC          0x53564453u   // "SVDS"
#define LPOSC_NOMINAL_HZ  32768u
// The always-on timer ticks from the LPOSC, untrimmed (+/-20 % per datasheet).
// Before sleeping it is measured against the crystal-driven system time (0.1 %
// resolution in 1 s); what may remain is charged to the clock as this many ms
// per second slept, so the next real sync is accepted even if it lands that far
// behind the estimate.
#define LPOSC_RESIDUAL_MS_PER_S 3u
// Longer than this and the timer value is not trusted (corrupt, not a nap).
#define DS_MAX_SLEPT_MS   (30ULL * 24 * 3600 * 1000)

static bool s_refused;   // the power manager rejected a request this power cycle

// Measure the LPOSC: run the timer from the nominal 32.768 kHz tick for ~1 s of
// real time; its count error is the frequency error. 1 ms in 1000 ms -> 0.1 %.
static uint32_t lposc_measure_hz(void) {
    powman_timer_set_1khz_tick_source_lposc();
    powman_timer_set_ms(0);
    if (!powman_timer_is_running()) powman_timer_start();
    absolute_time_t t0 = get_absolute_time();
    sleep_ms(1000);
    uint64_t elapsed_us = (uint64_t) absolute_time_diff_us(t0, get_absolute_time());
    uint64_t ticks_ms = powman_timer_get_ms();
    uint64_t hz = (uint64_t) LPOSC_NOMINAL_HZ * ticks_ms * 1000ULL / elapsed_us;
    // Outside the datasheet spread the measurement itself is suspect.
    if (hz < 24000 || hz > 42000) return LPOSC_NOMINAL_HZ;
    return (uint32_t) hz;
}

static void ds_forget(void) {
    powman_hw->scratch[0] = 0;
    powman_disable_all_wakeups();
}

bool power_deep_sleep_available(void) { return !s_refused; }

bool power_deep_sleep_off(const station_config_t *cfg, const savia_deep_sleep_ctx_t *ctx) {
    if (s_refused || !cfg || !ctx || ctx->now_wall_ms == 0 ||
        ctx->nap_s < SAVIA_DEEP_SLEEP_MIN_S) return false;

    // Calibrate the tick, then load the wall clock (advanced by the time the
    // calibration and the caller's radio shutdown took) into the timer.
    uint32_t hz = lposc_measure_hz();
    powman_timer_set_1khz_tick_source_lposc_with_hz(hz);
    uint64_t up = savia_uptime_ms();
    uint64_t wall = ctx->now_wall_ms + (up - ctx->uptime_ms);
    powman_timer_set_ms(wall);
    if (!powman_timer_is_running()) powman_timer_start();

    powman_hw->scratch[0] = DS_MAGIC;
    powman_hw->scratch[1] = (uint32_t) wall;
    powman_hw->scratch[2] = (uint32_t)(wall >> 32);
    powman_hw->scratch[3] = ctx->nap_s;
    powman_hw->scratch[4] = ctx->lora_last_attempt_s;
    powman_hw->scratch[5] = (uint32_t) ctx->last_daily_day;
    powman_hw->scratch[6] = ctx->clock_uncertainty_ms;
    powman_hw->scratch[7] = ctx->lora_last_soil_hour_s;

    // Wake sources: the alarm at the end of the nap, or the button held low.
    powman_disable_all_wakeups();
    powman_enable_alarm_wakeup_at_ms(wall + (uint64_t) ctx->nap_s * 1000ULL);
    powman_enable_gpio_wakeup(0, cfg->wake_button_gpio, /*edge=*/false, /*high=*/false);
    // An attached debugger would otherwise veto the power-off.
    powman_set_debug_power_request_ignored(true);

    // Sleep in P1.7 (everything off but the always-on domain); wake into P0.0
    // (everything on) so the boot ROM and this firmware run again from flash.
    powman_power_state off = POWMAN_POWER_STATE_NONE;
    powman_power_state on  = POWMAN_POWER_STATE_NONE;
    on = powman_power_state_with_domain_on(on, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
    on = powman_power_state_with_domain_on(on, POWMAN_POWER_DOMAIN_XIP_CACHE);
    on = powman_power_state_with_domain_on(on, POWMAN_POWER_DOMAIN_SRAM_BANK0);
    on = powman_power_state_with_domain_on(on, POWMAN_POWER_DOMAIN_SRAM_BANK1);
    if (!powman_configure_wakeup_state(off, on)) {
        LOG_WARN("power: power manager refused the wake state -> light naps this power cycle\n");
        s_refused = true;
        ds_forget();
        return false;
    }
    // Boot from flash on wake (no stored entry point).
    for (int i = 0; i < 4; i++) powman_hw->boot[i] = 0;

    LOG_INFO("power: off (P1.7) for %u s; lposc=%u Hz; wake on timer or GP%u\n",
             (unsigned) ctx->nap_s, (unsigned) hz, (unsigned) cfg->wake_button_gpio);
    stdio_flush();
    int rc = powman_set_power_state(off);
    if (rc != PICO_OK) {
        LOG_WARN("power: power-off request rejected (%d) -> light naps this power cycle\n", rc);
        s_refused = true;
        ds_forget();
        return false;
    }
    // The switch happens once the core sleeps; on success this loop never ends.
    for (int i = 0; i < 100; i++) { __wfi(); sleep_ms(10); }
    LOG_WARN("power: still running after the power-off request -> light naps this power cycle\n");
    s_refused = true;
    ds_forget();
    return false;
}

void power_deep_wake_info(savia_deep_wake_t *out) {
    memset(out, 0, sizeof *out);
    out->last_daily_day = -1;
    if (powman_hw->scratch[0] != DS_MAGIC) return;
    uint64_t entry = (uint64_t) powman_hw->scratch[1] | ((uint64_t) powman_hw->scratch[2] << 32);
    powman_hw->scratch[0] = 0;   // consumed: a boot loop must not replay it
    // The GPIO wake latches its status; the alarm wake leaves it clear.
    bool button = (powman_hw->pwrup[0] & POWMAN_PWRUP0_STATUS_BITS) != 0;
    powman_disable_all_wakeups();
    powman_clear_alarm();
    if (!powman_timer_is_running()) return;   // no timer, no clock: cold start
    uint64_t now = powman_timer_get_ms();
    if (now < entry || now - entry > DS_MAX_SLEPT_MS) return;
    out->resumed = true;
    out->button = button;
    out->now_wall_ms = now;
    out->slept_ms = (uint32_t)(now - entry);
    out->lora_last_attempt_s = powman_hw->scratch[4];
    out->last_daily_day = (int32_t) powman_hw->scratch[5];
    out->lora_last_soil_hour_s = powman_hw->scratch[7];
    out->clock_uncertainty_ms = powman_hw->scratch[6] +
                                (out->slept_ms / 1000u) * LPOSC_RESIDUAL_MS_PER_S;
}

#else   // RP2040: no power manager -> the light nap is the only sleep.

bool power_deep_sleep_available(void) { return false; }

bool power_deep_sleep_off(const station_config_t *cfg, const savia_deep_sleep_ctx_t *ctx) {
    (void) cfg; (void) ctx;
    return false;
}

void power_deep_wake_info(savia_deep_wake_t *out) {
    memset(out, 0, sizeof *out);
    out->last_daily_day = -1;
}

#endif
