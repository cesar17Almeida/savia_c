#include "savia/lora.h"
#include "savia/lora_codec.h"
#include "savia/weather.h"
#include "savia/clock.h"
#include "savia/storage.h"
#include "savia/log.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>

// LoRaWAN node via a Grove Wio-E5 over UART (AT commands). The module runs the
// LoRaWAN MAC; we drive it. Mirrors savia_py's WioE5Modem + lora task:
//   - uplink (Class A): the HS30 forecast min -- mainly to open the RX windows;
//   - downlink: clock + air-temperature forecast -> clock_set + weather_set,
//     so the LSTM gets its TA window even with no phone around.
//
// OTAA credentials and link params are PLACEHOLDERS until the device is
// registered in TTN (Spain -> EU868). Fill these in after registration; the
// counterpart backend (TTN API + Open-Meteo) is a separate phase-2 piece.
#define LORA_DEV_EUI  "0000000000000000"
#define LORA_APP_EUI  "0000000000000000"   // a.k.a. JoinEUI
#define LORA_APP_KEY  "00000000000000000000000000000000"
#define LORA_REGION   "EU868"              // AT+DR=<region> band (Spain)
#define LORA_FPORT    8                    // application FPort (1..223)
#define LORA_BAUD     9600

#define LORA_JOIN_TIMEOUT_MS    12000u
#define LORA_UPLINK_TIMEOUT_MS  15000u
// One uplink (and at most one downlink) per period. TTN fair-use is ~10
// downlinks/day, so 6 h -> ~4/day. Also rate-limits join retries.
#define LORA_PERIOD_MS          (6u * 3600u * 1000u)

typedef enum { AT_OK, AT_FAIL, AT_TIMEOUT } at_status_t;

static uart_inst_t *s_uart;
static bool     s_ready;            // UART up + module configured
static bool     s_joined;
static bool     s_attempted;        // a cycle has run (so the first one is immediate)
static uint32_t s_last_attempt_ms;  // uptime ms of the last cycle attempt

// --- hex helpers ------------------------------------------------------------

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c = (char) tolower((unsigned char) c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int hex_to_bytes(const char *hex, uint8_t *out, size_t cap) {
    size_t len = strlen(hex);
    if (len % 2 || len / 2 > cap) return -1;
    for (size_t i = 0; i < len / 2; i++) {
        int hi = hexval(hex[2 * i]), lo = hexval(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t) ((hi << 4) | lo);
    }
    return (int) (len / 2);
}

static void bytes_to_hex(const uint8_t *b, size_t n, char *out) {
    static const char H[] = "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++) { out[2 * i] = H[b[i] >> 4]; out[2 * i + 1] = H[b[i] & 0xF]; }
    out[2 * n] = '\0';
}

// Pull the hex string out of a downlink line (spaces stripped). Tolerant of the
// firmware variants `RX: "AA BB"` and `RX "AABB"`; RXWIN1/RSSI lines have no
// quote so they fall through. Mirrors the lenient token-matching in savia_py.
static void extract_rx_hex(const char *line, char *out, size_t cap) {
    const char *p = strstr(line, "RX");
    if (!p || !(p = strchr(p, '"'))) return;
    p++;
    size_t n = 0;
    for (; *p && *p != '"'; p++) {
        if (isxdigit((unsigned char) *p) && n + 1 < cap) out[n++] = *p;
    }
    out[n] = '\0';
}

// --- UART line I/O ----------------------------------------------------------

// uart0/uart1 selection from the configured TX pin (standard RP2040/RP2350 map).
static uart_inst_t *uart_for_tx(uint8_t tx) {
    switch (tx) {
        case 0: case 12: case 16: case 28: return uart0;
        default:                           return uart1;   // incl. default GPIO4/5
    }
}

// Read one '\n'-terminated line (CR stripped, blanks skipped). Returns the
// length, or -1 if the deadline passed before a full line arrived.
static int read_line(char *buf, size_t cap, absolute_time_t deadline) {
    size_t n = 0;
    for (;;) {
        int64_t rem = absolute_time_diff_us(get_absolute_time(), deadline);
        if (rem <= 0) return -1;
        uint32_t slice = rem > 20000 ? 20000u : (uint32_t) rem;
        if (!uart_is_readable_within_us(s_uart, slice)) continue;
        char c = (char) uart_getc(s_uart);
        if (c == '\r') continue;
        if (c == '\n') { if (n == 0) continue; buf[n] = '\0'; return (int) n; }
        if (n + 1 < cap) buf[n++] = c;
    }
}

// Send `cmd`, then read replies until an `until`/`fail` token or timeout. When
// `rx_hex` is given, any downlink hex (RX:"...") seen along the way is captured.
static at_status_t at_exec(const char *cmd,
                           const char *const *until, int n_until,
                           const char *const *fail, int n_fail,
                           uint32_t timeout_ms, char *rx_hex, size_t rx_cap) {
    uart_puts(s_uart, cmd);
    uart_puts(s_uart, "\r\n");
    LOG_DEBUG("LoRa -> %s\n", cmd);
    if (rx_hex && rx_cap) rx_hex[0] = '\0';

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    char line[256];
    while (read_line(line, sizeof line, deadline) > 0) {
        LOG_DEBUG("LoRa <- %s\n", line);
        if (rx_hex && rx_cap) extract_rx_hex(line, rx_hex, rx_cap);
        for (int i = 0; i < n_fail; i++)  if (strstr(line, fail[i]))  return AT_FAIL;
        for (int i = 0; i < n_until; i++) if (strstr(line, until[i])) return AT_OK;
    }
    return AT_TIMEOUT;
}

// --- join / configure -------------------------------------------------------

static bool lora_configure(void) {
    static const char *ok[]   = { "OK" };
    static const char *fail[] = { "ERROR" };
    char at[64];

    if (at_exec("AT", ok, 1, fail, 1, 1000, NULL, 0) == AT_TIMEOUT) {
        LOG_WARN("LoRa: no AT response (module wired/powered?)\n");
        return false;
    }
    at_exec("AT+MODE=LWOTAA", ok, 1, fail, 1, 3000, NULL, 0);
    snprintf(at, sizeof at, "AT+ID=DevEui,\"%s\"", LORA_DEV_EUI);
    at_exec(at, ok, 1, fail, 1, 3000, NULL, 0);
    snprintf(at, sizeof at, "AT+ID=AppEui,\"%s\"", LORA_APP_EUI);
    at_exec(at, ok, 1, fail, 1, 3000, NULL, 0);
    snprintf(at, sizeof at, "AT+KEY=APPKEY,\"%s\"", LORA_APP_KEY);
    at_exec(at, ok, 1, fail, 1, 3000, NULL, 0);
    at_exec("AT+DR=" LORA_REGION, ok, 1, fail, 1, 3000, NULL, 0);
    return true;
}

static bool lora_join(void) {
    static const char *until[] = { "Network joined", "Joined already", "+JOIN: Done" };
    static const char *fail[]  = { "Join failed", "+JOIN: Failed" };
    return at_exec("AT+JOIN", until, 3, fail, 2, LORA_JOIN_TIMEOUT_MS, NULL, 0) == AT_OK;
}

// --- forecast telemetry for the uplink --------------------------------------

// Min of the still-upcoming HS30 forecast, for the uplink payload. Empty on the
// Pico WH (inference is off-device) -> reported as unknown.
static bool forecast_min_upcoming(uint64_t now_ms, float *out_min) {
    savia_prediction_t buf[32];
    size_t n = storage_query_pred(now_ms, UINT64_MAX, 0, buf, 32);
    bool found = false;
    float m = 0.0f;
    for (size_t i = 0; i < n; i++) {
        if (strcmp(buf[i].kind, "hs30_forecast") != 0) continue;
        if (!found || buf[i].value < m) { m = buf[i].value; found = true; }
    }
    if (found) *out_min = m;
    return found;
}

static uint64_t wall_now(uint32_t uptime_ms) {
    return clock_is_set() ? clock_now(uptime_ms) : 0;
}

// --- public API -------------------------------------------------------------

bool lora_init(const station_config_t *cfg) {
    if (!cfg || !cfg->lora_enabled) return false;

    s_uart = uart_for_tx(cfg->lora_uart_tx_gpio);
    uart_init(s_uart, LORA_BAUD);
    uart_set_format(s_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(s_uart, true);
    gpio_set_function(cfg->lora_uart_tx_gpio, GPIO_FUNC_UART);
    gpio_set_function(cfg->lora_uart_rx_gpio, GPIO_FUNC_UART);

    s_ready = lora_configure();
    s_joined = s_ready && lora_join();
    s_attempted = false;
    LOG_INFO("LoRa: init uart%d tx=%u rx=%u ready=%d joined=%d\n",
             s_uart == uart0 ? 0 : 1, cfg->lora_uart_tx_gpio,
             cfg->lora_uart_rx_gpio, s_ready, s_joined);
    return s_joined;
}

bool lora_cycle(void) {
    if (!s_ready) return false;
    uint32_t up = to_ms_since_boot(get_absolute_time());

    // One uplink per period (TTN fair use); the first cycle runs immediately.
    if (s_attempted && (up - s_last_attempt_ms) < LORA_PERIOD_MS) return false;
    s_attempted = true;
    s_last_attempt_ms = up;

    if (!s_joined) {                       // retry the OTAA join (e.g. after a drop)
        s_joined = lora_join();
        if (!s_joined) { LOG_WARN("LoRa: join failed, retry next period\n"); return false; }
        LOG_INFO("LoRa: joined network\n");
    }

    // Uplink: HS30 forecast min (or unknown). Its real job is to open RX windows.
    float hs30 = 0.0f;
    bool has = forecast_min_upcoming(wall_now(up), &hs30);
    uint8_t payload[LORA_UPLINK_LEN];
    size_t plen = lora_encode_uplink(has, hs30, payload, sizeof payload);
    char tx_hex[2 * LORA_UPLINK_LEN + 1];
    bytes_to_hex(payload, plen, tx_hex);

    static const char *ok[]   = { "OK" };
    static const char *okfail[] = { "ERROR" };
    char cmd[48];
    snprintf(cmd, sizeof cmd, "AT+PORT=%d", LORA_FPORT);
    at_exec(cmd, ok, 1, okfail, 1, 3000, NULL, 0);

    static const char *until[] = { "Done" };
    static const char *fail[]  = { "ERROR", "Please join", "Not join" };
    char rx_hex[2 * LORA_DOWNLINK_MAX + 1];
    snprintf(cmd, sizeof cmd, "AT+CMSGHEX=\"%s\"", tx_hex);
    at_status_t st = at_exec(cmd, until, 1, fail, 3, LORA_UPLINK_TIMEOUT_MS,
                             rx_hex, sizeof rx_hex);
    if (st == AT_FAIL) {
        LOG_WARN("LoRa: uplink rejected, will re-join\n");
        s_joined = false;
        return false;
    }
    if (st == AT_TIMEOUT) { LOG_WARN("LoRa: uplink timeout\n"); return false; }
    if (rx_hex[0] == '\0') { LOG_INFO("LoRa: uplink sent, no downlink\n"); return false; }

    // Downlink: decode -> set the clock + cache the TA forecast for the LSTM.
    uint8_t dl[LORA_DOWNLINK_MAX];
    int dn = hex_to_bytes(rx_hex, dl, sizeof dl);
    lora_downlink_t w;
    if (dn < 0 || !lora_decode_downlink(dl, (size_t) dn, &w)) {
        LOG_WARN("LoRa: bad downlink (%d B)\n", dn);
        return false;
    }
    uint32_t now_up = to_ms_since_boot(get_absolute_time());
    if (w.has_time) clock_set(w.time_ms, now_up);
    weather_set(w.past_ta, w.n_past, w.future_ta, w.n_future, wall_now(now_up));
    LOG_INFO("LoRa downlink: %u past + %u future TA%s\n",
             w.n_past, w.n_future, w.has_time ? ", clock set" : "");
    return true;
}
