// Hardware probe for the Savia station: checks whether the LoRa Wio-E5 and the
// AquaCheck SDI-12 probe are physically connected and responding.
//
//   LoRa Wio-E5 : UART0, GP16=Pico TX -> module RX, GP17=Pico RX <- module TX,
//                 9600 8N1. Sends "AT" and expects "+AT: OK".
//   SDI-12      : single-wire half-duplex on GP18, 1200 7E1 INVERTED (marking/
//                 idle/logic-1 = LOW; spacing/start/logic-0 = HIGH). Probe powered
//                 at 3.3 V so the line marks at ~3.1 V -- no level shifter.
//                 Sends "?!"/"0I!"; edges>0 means the probe drove the line.
//
// Output goes over USB serial. THROWAWAY bring-up tool; not the firmware.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/cyw43_arch.h"

// ---- Onboard LED heartbeat (alive / not in deep sleep) --------------------
// On the Pico W the onboard LED hangs off the CYW43 chip, not a plain GPIO.
static bool g_led_ok = false;       // cyw43 init succeeded
static bool g_led_state = false;
static void led(bool on) { if (g_led_ok) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }
static void blink_wait(uint32_t ms) {           // ~2 Hz blink while waiting -> clearly alive
    for (uint32_t t = 0; t < ms; t += 250) { g_led_state = !g_led_state; led(g_led_state); sleep_ms(250); }
}

// ---- LoRa Wio-E5 on UART0 -------------------------------------------------
#define LORA_UART uart0
#define LORA_TX   16
#define LORA_RX   17
#define LORA_BAUD 9600

static void lora_init(void) {
    uart_init(LORA_UART, LORA_BAUD);
    gpio_set_function(LORA_TX, GPIO_FUNC_UART);
    gpio_set_function(LORA_RX, GPIO_FUNC_UART);
    uart_set_format(LORA_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LORA_UART, true);
}

static int lora_cmd(const char *cmd, char *buf, int max, uint32_t timeout_ms) {
    while (uart_is_readable(LORA_UART)) (void) uart_getc(LORA_UART);   // drain
    uart_puts(LORA_UART, cmd);
    uart_puts(LORA_UART, "\r\n");
    int n = 0;
    absolute_time_t dl = make_timeout_time_ms(timeout_ms);
    while (n < max - 1 && absolute_time_diff_us(get_absolute_time(), dl) > 0) {
        if (uart_is_readable_within_us(LORA_UART, 5000)) buf[n++] = (char) uart_getc(LORA_UART);
    }
    buf[n] = 0;
    return n;
}

static void lora_try(const char *cmd) {
    char buf[160];
    int n = lora_cmd(cmd, buf, sizeof buf, 800);
    printf("  >>> %-8s resp=%dB: \"", cmd, n);
    for (int i = 0; i < n; i++) { char c = buf[i]; putchar((c >= 32 && c < 127) ? c : '.'); }
    printf("\"\n");
}

// Read GP17 (RX) idle level with pull-up then pull-down. A powered module's TX
// actively holds the line HIGH (UART idle), so both reads agree; a floating line
// (no TX wire / unpowered module) follows the pull.
static void rx_line_check(void) {
    gpio_set_function(LORA_RX, GPIO_FUNC_SIO);
    gpio_set_dir(LORA_RX, GPIO_IN);
    gpio_pull_up(LORA_RX);     busy_wait_us(3000); int up   = gpio_get(LORA_RX);
    gpio_pull_down(LORA_RX);   busy_wait_us(3000); int down = gpio_get(LORA_RX);
    gpio_disable_pulls(LORA_RX);
    printf("  RX(GP17) reposo: pull-up=%d pull-down=%d -> ", up, down);
    if (up == down) printf("FIJADO a %d -> el TX del modulo ESTA conectado y alimentado %s\n",
                           up, up ? "(idle alto, bien)" : "(raro: bajo)");
    else            printf("FLOTANTE -> el TX del modulo NO llega a GP17, o el modulo NO esta alimentado\n");
    gpio_set_function(LORA_RX, GPIO_FUNC_UART);
}

static void lora_probe(void) {
    char buf[160];
    printf("\n--- LoRa Wio-E5 (UART0 GP16->RX / GP17<-TX @9600) ---\n");
    rx_line_check();
    int n = lora_cmd("AT", buf, sizeof buf, 800);
    int has_plus = 0;                       // a real Wio-E5 replies "+AT: OK"
    for (int i = 0; i < n; i++) if (buf[i] == '+') { has_plus = 1; break; }
    int is_echo = (n >= 2 && buf[0] == 'A' && buf[1] == 'T' && !has_plus);
    printf("  AT -> %dB raw: ", n);
    for (int i = 0; i < n; i++) printf("%02X ", (uint8_t) buf[i]);
    printf("(\"");
    for (int i = 0; i < n; i++) { char c = buf[i]; putchar((c >= 32 && c < 127) ? c : '.'); }
    printf("\")\n");
    if (has_plus)     printf("  => +AT:OK -> MODULO REAL CONECTADO Y VIVO  [OK]\n");
    else if (is_echo) printf("  => ECO de mi propio comando (no +AT:OK) -> LOOPBACK: GP16<->GP17 unidos, o modulo fuera del bus  [FALLO]\n");
    else if (n == 0)  printf("  => sin respuesta -> modulo sin alimentar, o GP17(RX) no recibe del TX del modulo  [FALLO]\n");
    else              printf("  => respuesta inesperada -> posible baud != 9600 o cableado  [REVISAR]\n");
    lora_try("AT+VER");
}

// ---- SDI-12 AquaCheck on GP18 (bit-bang) ----------------------------------
#define SDI_PIN  18
#define BIT_US   833
#define HALF_US  417

static void tx_char(char ch) {
    uint8_t d = (uint8_t) ch & 0x7F; int ones = 0;
    gpio_put(SDI_PIN, 1); busy_wait_us(BIT_US);
    for (int i = 0; i < 7; i++) {
        int bit = (d >> i) & 1; if (bit) ones++;
        gpio_put(SDI_PIN, bit ? 0 : 1); busy_wait_us(BIT_US);
    }
    gpio_put(SDI_PIN, (ones & 1) ? 0 : 1); busy_wait_us(BIT_US);
    gpio_put(SDI_PIN, 0); busy_wait_us(BIT_US);
}

static void sdi_send(const char *s) {
    gpio_set_dir(SDI_PIN, GPIO_OUT);
    gpio_put(SDI_PIN, 1); busy_wait_us(13000);
    gpio_put(SDI_PIN, 0); busy_wait_us(9000);
    while (*s) tx_char(*s++);
    gpio_set_dir(SDI_PIN, GPIO_IN);
    gpio_pull_down(SDI_PIN);
}

// Oversampled RX: capture the line at ~8x the bit rate, decode offline. Robust
// against the drift a real-time bit sampler shows on long frames.
#define OS_US   104        // sample period (833us bit / 8)
#define OS_BIT  8          // samples per bit
#define OS_MAX  6000       // ~624 ms capture
static uint8_t g_os[OS_MAX];

static int sdi_capture(uint32_t window_ms) {
    int total = (int) ((uint32_t) window_ms * 1000u / OS_US);
    if (total > OS_MAX) total = OS_MAX;
    absolute_time_t t = get_absolute_time();
    for (int i = 0; i < total; i++) {
        g_os[i] = (uint8_t) gpio_get(SDI_PIN);
        t = delayed_by_us(t, OS_US);
        busy_wait_until(t);          // fixed grid -> no cumulative drift
    }
    return total;
}

// SDI-12 frame: inverted, idle=LOW(marking), start=HIGH(spacing), 7 data bits
// LSB-first (LOW=logic1), even parity, stop=LOW. Resync on every start edge.
static int sdi_decode(int ns, char *out, int max) {
    int n = 0, i = 0;
    while (n < max) {
        while (i < ns && g_os[i] != 0) i++;     // skip HIGH
        while (i < ns && g_os[i] == 0) i++;     // skip LOW idle -> first HIGH = start edge
        if (i >= ns) break;
        int start = i;
        uint8_t c = 0; int bad = 0;
        for (int b = 0; b < 7; b++) {
            int idx = start + OS_BIT * b + OS_BIT + OS_BIT / 2;   // centre of data bit b
            if (idx >= ns) { bad = 1; break; }
            if (g_os[idx] == 0) c |= (1u << b);                  // LOW = logic 1
        }
        if (bad) break;
        out[n++] = (char) (c & 0x7F);
        i = start + 10 * OS_BIT - OS_BIT / 2;   // jump into the stop bit, then find next start
    }
    return n;
}

static void sdi_try(const char *cmd, uint32_t win_ms) {
    char out[80];
    sdi_send(cmd);
    int ns = sdi_capture(win_ms);
    int n = sdi_decode(ns, out, sizeof out);
    printf("  %-6s -> %2dB: \"", cmd, n);
    for (int i = 0; i < n; i++) { char ch = out[i]; putchar((ch >= 32 && ch < 127) ? ch : '.'); }
    printf("\"  hex:");
    for (int i = 0; i < n; i++) printf(" %02X", (uint8_t) out[i]);
    printf("\n");
}

static void sdi_measure(const char *mcmd) {
    sdi_try(mcmd, 500);        // "0tttn": ttt = delay (s), n = number of values
    sleep_ms(2000);            // wait out the measurement
    sdi_try("0D0!", 600);
    sdi_try("0D1!", 600);
    sdi_try("0D2!", 600);
}

static void sdi_probe(void) {
    printf("\n--- SDI-12 AquaCheck (GP18) -- barrido de comandos ---\n");
    sdi_try("?!",   500);      // address query
    sdi_try("0!",   500);      // ack active
    sdi_try("0I!",  800);      // identification
    sdi_try("0R0!", 800);      // read config (V43+): version + #sensors + length(cm)
    sdi_try("0X#!", 800);      // read config (V40+): "a#nLx" -> #sensors + length CODE Lx
    printf("  [HUMEDAD] 0M! + 0D0..2!\n");
    sdi_measure("0M!");
    printf("  [TEMPERATURA] 0M1! + 0D0..2!\n");
    sdi_measure("0M1!");
    sdi_try("0C!",  800);      // concurrent measurement
}

int main(void) {
    stdio_init_all();
    g_led_ok = (cyw43_arch_init() == 0);   // onboard LED lives on the CYW43 chip
    lora_init();
    gpio_init(SDI_PIN); gpio_set_dir(SDI_PIN, GPIO_OUT); gpio_put(SDI_PIN, 0);
    sleep_ms(2500);     // settle + USB attach
    printf("\n================ HW PROBE: LoRa Wio-E5 + SDI-12 ================\n");
    printf("LED de la placa parpadeando = firmware VIVO (no en sueno profundo).\n");
    while (true) {
        led(true);              // solid while probing
        lora_probe();
        sdi_probe();
        blink_wait(3000);       // blink during the gap -> visibly alive
    }
}
