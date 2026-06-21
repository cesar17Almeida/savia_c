#include "savia/power.h"
#include "pico/stdlib.h"

// The button is wired to GND with an internal pull-up, so it reads low when
// pressed (active-low).
void power_init(const station_config_t *cfg) {
    gpio_init(cfg->wake_button_gpio);
    gpio_set_dir(cfg->wake_button_gpio, GPIO_IN);
    gpio_pull_up(cfg->wake_button_gpio);
}

savia_wake_reason_t power_deep_sleep(const station_config_t *cfg, uint32_t seconds) {
    // TODO(hw): real DORMANT. Switch clocks to the low-power source and arm the
    // wake button as a dormant GPIO wake source (RP2040: clocks + xosc dormant;
    // RP2350: powman + the same GPIO wake). The radio is already powered down by
    // ble_radio_suspend() before we get here, which is the bulk of the saving.
    //
    // For now a light-sleep stand-in (sleep_ms idles the core via WFE) so the
    // supervisor logic runs end-to-end.
    const uint32_t step_ms = 50;
    uint32_t elapsed = 0;
    while (elapsed < seconds * 1000u) {
        if (!gpio_get(cfg->wake_button_gpio)) {  // active-low press
            return SAVIA_WAKE_BUTTON;
        }
        sleep_ms(step_ms);
        elapsed += step_ms;
    }
    return SAVIA_WAKE_TIMER;
}
