// Onboard-LED status indicator. The Pico W LED is wired to the CYW43 chip, so a
// hardware-timer IRQ can't touch it safely; instead we run a cyw43 async_context
// worker. It keeps blinking even while the supervisor loop is blocked in
// ble_poll() or napping, and only runs while the radio is powered (on a real
// deep-sleep suspend the chip -- and the LED -- are off).

#include "savia/status_led.h"

#if SAVIA_ENABLE_BLE

#include "savia/ble.h"
#include "pico/cyw43_arch.h"

#define TICK_MS 100

static async_at_time_worker_t s_worker;
static uint32_t s_tick;            // 100 ms ticks

static void led_work(async_context_t *ctx, async_at_time_worker_t *w) {
    s_tick++;
    bool on;
    if (ble_is_connected())        on = true;                 // solid = paired
    else if (ble_is_advertising()) on = (s_tick % 10) < 2;    // ~200 ms flash every 1 s
    else                           on = (s_tick % 30) < 2;    // ~200 ms flash every 3 s
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    async_context_add_at_time_worker_in_ms(ctx, w, TICK_MS);
}

void status_led_init(void) {
    s_worker.do_work = led_work;
    async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &s_worker, TICK_MS);
}

#else  // SAVIA_ENABLE_BLE == 0

void status_led_init(void) {}      // no CYW43 radio -> no onboard LED

#endif
