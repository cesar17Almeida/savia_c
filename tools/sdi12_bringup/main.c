// SDI-12 bring-up test for an AquaCheck probe on GP18 (Raspberry Pi Pico W).
//
// Single-wire half-duplex, 1200 baud, 7 data bits, even parity, 1 stop bit,
// INVERTED levels: marking/idle/logic-1 = LOW (0 V); spacing/start/logic-0 =
// HIGH (~3.1 V). The probe is powered at 3.3 V so the data line marks at
// ~3.1 V -- safe for the 3.3 V GPIO, no level shifter.
//
// It sends "?!" (address query) and "0I!" (identify) in a loop and prints,
// over USB serial:
//   edges = line transitions seen in the RX window (primary "is it alive?"
//           signal -- robust, independent of the UART decode)
//   ascii/hex = best-effort decode of the reply bytes
//
// Wiring: Brown->3V3(pin36), Black->GND(pin38), Blue(DATA)->GP18(pin24),
//         White (RS485-A) left unconnected.
//
// THROWAWAY bring-up tool: not part of the firmware. The real SDI-12 driver
// (PIO-based) still lives as TODO(hw) in src/drivers/sensor_sdi12.c.

#include <stdio.h>
#include "pico/stdlib.h"

#define SDI_PIN  18
#define BIT_US   833      // 1 / 1200 baud
#define HALF_US  417

static void tx_char(char ch) {
    uint8_t d = (uint8_t)ch & 0x7F;     // 7 data bits
    int ones = 0;
    gpio_put(SDI_PIN, 1); busy_wait_us(BIT_US);          // start bit  = spacing = HIGH
    for (int i = 0; i < 7; i++) {                         // data, LSB first, inverted
        int bit = (d >> i) & 1; if (bit) ones++;
        gpio_put(SDI_PIN, bit ? 0 : 1);                  // logic 1 -> LOW, logic 0 -> HIGH
        busy_wait_us(BIT_US);
    }
    gpio_put(SDI_PIN, (ones & 1) ? 0 : 1);               // even parity (inverted)
    busy_wait_us(BIT_US);
    gpio_put(SDI_PIN, 0); busy_wait_us(BIT_US);          // stop bit = marking = LOW
}

static void send_cmd(const char *s) {
    gpio_set_dir(SDI_PIN, GPIO_OUT);
    gpio_put(SDI_PIN, 1); busy_wait_us(13000);           // break  (spacing >= 12 ms)
    gpio_put(SDI_PIN, 0); busy_wait_us(9000);            // marking (>= 8.33 ms)
    while (*s) tx_char(*s++);
    gpio_set_dir(SDI_PIN, GPIO_IN);                      // release the line to receive
    gpio_pull_down(SDI_PIN);                             // hold marking (LOW) when idle
}

// Wait for the reply, decoding best-effort and counting every line transition.
static int recv(char *buf, int max, uint32_t window_ms, int *edges_out) {
    int n = 0, edges = 0, last = gpio_get(SDI_PIN);
    absolute_time_t dl = make_timeout_time_ms(window_ms);
    while (n < max) {
        int cur;
        do {                                             // poll for start bit (line HIGH)
            if (absolute_time_diff_us(get_absolute_time(), dl) <= 0) { *edges_out = edges; return n; }
            cur = gpio_get(SDI_PIN);
            if (cur != last) { edges++; last = cur; }
        } while (cur == 0);
        busy_wait_us(BIT_US + HALF_US);                  // -> centre of data bit 0
        uint8_t c = 0;
        for (int i = 0; i < 7; i++) {
            if (gpio_get(SDI_PIN) == 0) c |= (1u << i);  // LOW = logic 1
            busy_wait_us(BIT_US);
        }
        busy_wait_us(BIT_US + HALF_US);                  // step past parity into stop
        buf[n++] = (char)(c & 0x7F);
        last = gpio_get(SDI_PIN);
    }
    *edges_out = edges; return n;
}

static void try_cmd(const char *cmd) {
    char buf[96]; int edges = 0;
    printf("\n>>> TX \"%s\"\n", cmd);
    send_cmd(cmd);
    int n = recv(buf, sizeof buf, 1000, &edges);
    printf("    edges=%d  bytes=%d\n", edges, n);
    printf("    ascii: \"");
    for (int i = 0; i < n; i++) { char c = buf[i]; putchar((c >= 32 && c < 127) ? c : '.'); }
    printf("\"\n    hex:   ");
    for (int i = 0; i < n; i++) printf("%02X ", (uint8_t)buf[i]);
    printf("\n");
}

int main(void) {
    stdio_init_all();
    gpio_init(SDI_PIN);
    gpio_set_dir(SDI_PIN, GPIO_OUT);
    gpio_put(SDI_PIN, 0);                                // idle = marking (LOW)
    sleep_ms(2500);                                      // let the probe settle (>1 s) + USB attach
    printf("\n=== SDI-12 bring-up on GP18 (1200 7E1 inverted, half-duplex) ===\n");
    printf("edges>0 means the probe is driving the line (alive). edges=0 -> check wiring/power/address.\n");
    while (true) {
        try_cmd("?!");    // -> expect address "0\r\n"
        sleep_ms(1500);
        try_cmd("0I!");   // -> expect e.g. "013ACCSDI..." / "013ACHSDI..."
        sleep_ms(2500);
    }
}
