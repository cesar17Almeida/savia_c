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
// OTAA credentials for the TTN end device (EU868, eu1 cluster). They are SECRETS and
// must never be committed: put them in a local .env (see .env.example), which the
// Makefile forwards to CMake as -DSAVIA_LORA_*. Without it the build falls back to the
// all-zeros placeholders below, which compile and run but never join a real network.
#ifndef SAVIA_LORA_DEV_EUI
#define SAVIA_LORA_DEV_EUI "0000000000000000"
#endif
#ifndef SAVIA_LORA_APP_EUI
#define SAVIA_LORA_APP_EUI "0000000000000000"   // a.k.a. JoinEUI; all-zeros is the TTN default
#endif
#ifndef SAVIA_LORA_APP_KEY
#define SAVIA_LORA_APP_KEY "00000000000000000000000000000000"
#endif

#define LORA_DEV_EUI  SAVIA_LORA_DEV_EUI
#define LORA_APP_EUI  SAVIA_LORA_APP_EUI
#define LORA_APP_KEY  SAVIA_LORA_APP_KEY
#define LORA_REGION   "EU868"              // AT+DR=<region> band (Spain)
#define LORA_FPORT    8                    // application FPort (1..223)
#define LORA_BAUD     9600

#define LORA_JOIN_TIMEOUT_MS    12000u
#define LORA_UPLINK_TIMEOUT_MS  15000u
// One uplink (and at most one downlink) per cfg.lora_period_s (default 1 h). With a
// private gateway + paid plan the TTN fair-use cap doesn't apply; the only ceiling
// is the EU868 ~1% duty cycle, which hourly cycles clear easily. Also gates join retries.

typedef enum { AT_OK, AT_FAIL, AT_TIMEOUT } at_status_t;

static uart_inst_t *s_uart;
static bool     s_ready;            // UART up + module configured
static bool     s_joined;
static bool     s_attempted;        // a cycle has run (so the first one is immediate)
static uint32_t s_last_attempt_ms;  // uptime ms of the last cycle attempt
static uint8_t  s_tx, s_rx;         // GPIOs the module is currently wired to (0 = none)

// Last downlink signal measured from an uplink ACK (the only signal the node sees).
static bool     s_has_signal;
static int16_t  s_rssi_dbm;
static int16_t  s_snr_ddb;          // deci-dB (x10)
static uint64_t s_last_signal_ms;
static bool     s_signal_seen;      // set by at_exec when a line carries RSSI/SNR
static char     s_module[24];       // module AT+VER reply ("" until a successful AT)
static uint32_t s_seq;              // completed-ping counter (the app waits for a bump)
static lora_at_result_t s_at;       // last raw AT exchange (the BLE "AT terminal")

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

// Parse a signed integer at *p (no leading sign skipping). Advances nothing.
static int parse_signed(const char *p) {
    int sign = 1;
    if (*p == '-') { sign = -1; p++; }
    int v = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; }
    return sign * v;
}

// Pull RSSI (dBm) + SNR (deci-dB) out of a Wio-E5 ACK line, e.g.
// "+CMSGHEX: RXWIN1, RSSI -106, SNR 7" or "... RSSI:-106 SNR:7.0". Both numbers
// must be present. Returns true and fills *rssi / *snr_ddb on a match.
static bool extract_rssi_snr(const char *line, int *rssi, int *snr_ddb) {
    const char *pr = strstr(line, "RSSI");
    const char *ps = strstr(line, "SNR");
    if (!pr || !ps) return false;
    pr += 4;
    while (*pr && !(*pr == '-' || (*pr >= '0' && *pr <= '9'))) pr++;   // skip " :"
    ps += 3;
    while (*ps && !(*ps == '-' || (*ps >= '0' && *ps <= '9'))) ps++;
    *rssi = parse_signed(pr);
    // SNR may be fractional ("7.0", "-2.5") -> one decimal place into deci-dB.
    int sign = 1; const char *p = ps;
    if (*p == '-') { sign = -1; p++; }
    int whole = 0; while (*p >= '0' && *p <= '9') { whole = whole * 10 + (*p - '0'); p++; }
    int frac = 0; if (*p == '.' && p[1] >= '0' && p[1] <= '9') frac = p[1] - '0';
    *snr_ddb = sign * (whole * 10 + frac);
    return true;
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
    bool truncated = false;
    for (;;) {
        int64_t rem = absolute_time_diff_us(get_absolute_time(), deadline);
        if (rem <= 0) return -1;
        uint32_t slice = rem > 20000 ? 20000u : (uint32_t) rem;
        if (!uart_is_readable_within_us(s_uart, slice)) continue;
        char c = (char) uart_getc(s_uart);
        if (c == '\r') continue;
        if (c == '\n') {
            if (n == 0) continue;
            if (truncated) LOG_WARN("LoRa: line truncated to %u chars (buffer too small)\n", (unsigned) n);
            buf[n] = '\0';
            return (int) n;
        }
        if (n + 1 < cap) buf[n++] = c;   // keep room for the NUL
        else truncated = true;           // tail dropped; warn rather than silently lose payload
    }
}

// Send `cmd`, then read replies until an `until`/`fail` token or timeout. When
// `rx_hex` is given, any downlink hex (RX:"...") seen along the way is captured.
// RSSI/SNR on any line update the module's last-signal state (s_signal_seen).
static at_status_t at_exec(const char *cmd,
                           const char *const *until, int n_until,
                           const char *const *fail, int n_fail,
                           uint32_t timeout_ms, char *rx_hex, size_t rx_cap) {
    uart_puts(s_uart, cmd);
    uart_puts(s_uart, "\r\n");
    LOG_DEBUG("LoRa -> %s\n", cmd);
    if (rx_hex && rx_cap) rx_hex[0] = '\0';

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    // Sized for the largest spaced downlink line: prefix/quote + 2 hex chars and
    // one separating space per payload byte ("AA BB ..."), plus NUL slack.
    char line[3 * LORA_DOWNLINK_MAX + 64];
    while (read_line(line, sizeof line, deadline) > 0) {
        LOG_DEBUG("LoRa <- %s\n", line);
        if (rx_hex && rx_cap) extract_rx_hex(line, rx_hex, rx_cap);
        int rssi, snr_ddb;
        if (extract_rssi_snr(line, &rssi, &snr_ddb)) {
            s_rssi_dbm = (int16_t) rssi; s_snr_ddb = (int16_t) snr_ddb; s_signal_seen = true;
        }
        for (int i = 0; i < n_fail; i++)  if (strstr(line, fail[i]))  return AT_FAIL;
        for (int i = 0; i < n_until; i++) if (strstr(line, until[i])) return AT_OK;
    }
    return AT_TIMEOUT;
}

// Send a query and capture its most informative reply line (skipping a bare "OK").
// Used for AT+VER so the app can show what the module said back. NUL-terminated.
static void at_query(const char *cmd, char *out, size_t cap) {
    if (cap) out[0] = '\0';
    uart_puts(s_uart, cmd);
    uart_puts(s_uart, "\r\n");
    LOG_DEBUG("LoRa -> %s\n", cmd);
    absolute_time_t deadline = make_timeout_time_ms(1500);
    char line[64];
    while (read_line(line, sizeof line, deadline) > 0) {
        LOG_DEBUG("LoRa <- %s\n", line);
        if (line[0] && strcmp(line, "OK") != 0 && cap) {
            strncpy(out, line, cap - 1); out[cap - 1] = '\0';
        }
        if (strstr(line, "OK") || strstr(line, "ERROR")) break;
    }
}

// --- join / configure -------------------------------------------------------

static bool lora_configure(void) {
    static const char *ok[]   = { "OK" };
    static const char *fail[] = { "ERROR" };
    char at[64];

    if (at_exec("AT", ok, 1, fail, 1, 1000, NULL, 0) == AT_TIMEOUT) {
        LOG_WARN("LoRa: no AT response (check RX/TX wiring + VCC)\n");
        return false;
    }
    at_query("AT+VER", s_module, sizeof s_module);   // proof the module talks back
    LOG_INFO("LoRa: module responds, AT+VER: %s\n", s_module[0] ? s_module : "?");
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

// Bring the UART up on tx/rx and configure the module (no join). Idempotent:
// re-opens if the pins changed. Sets s_ready.
static bool lora_open(uint8_t tx, uint8_t rx) {
    s_uart = uart_for_tx(tx);
    uart_init(s_uart, LORA_BAUD);
    uart_set_format(s_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(s_uart, true);
    gpio_set_function(tx, GPIO_FUNC_UART);
    gpio_set_function(rx, GPIO_FUNC_UART);
    s_tx = tx; s_rx = rx;
    s_module[0] = '\0';            // drop any stale version until this AT succeeds
    // Warmup: the Wio-E5 reliably swallows the FIRST command after the UART opens
    // (observed on-board: first AT always times out, second succeeds). Send a
    // throwaway AT and ignore the outcome so lora_configure starts clean.
    {
        static const char *ok[] = { "OK" };
        static const char *fail[] = { "ERROR" };
        (void) at_exec("AT", ok, 1, fail, 1, 300, NULL, 0);
    }
    s_ready = lora_configure();
    LOG_INFO("LoRa: open uart%d tx=%u rx=%u ready=%d\n",
             s_uart == uart0 ? 0 : 1, tx, rx, s_ready);
    return s_ready;
}

// --- uplink payload builders --------------------------------------------------

// Pending CFG_ACK for the next cycle (queued by the supervisor after applying a
// CONFIG downlink) and the last CONFIG TLV received (handed to the supervisor).
static bool    s_ack_pending;
static uint8_t s_ack_applied, s_ack_rejected;
static uint8_t s_cfg_tlv[LORA_DOWNLINK_MAX];
static size_t  s_cfg_tlv_len;          // 0 = none pending
// Coords already uplinked this power cycle (re-sent when they change).
static bool    s_coords_sent;
static int32_t s_coords_lat, s_coords_lon;
// Newest soil hour already uplinked (avoid resending the same aggregates).
static uint32_t s_last_soil_hour_s;

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

// Build SOIL records (FORWARD): hourly aggregates newer than the last uplinked
// hour, merged per hour from the (kind, depth) buckets. Returns record count.
static uint8_t build_soil_recs(uint64_t now_ms, lora_soil_rec_t *recs) {
    if (!now_ms) return 0;                        // no wall clock yet
    uint64_t from = (uint64_t)(s_last_soil_hour_s + 1) * 1000u;
    uint64_t lookback = now_ms > 5u * 3600000u ? now_ms - 5u * 3600000u : 0;
    if (from < lookback) from = lookback;         // cap the backlog window
    savia_aggregate_t aggs[24];
    size_t na = storage_aggregate_hourly(from, now_ms, 0, aggs, 24);
    uint8_t n = 0;
    for (size_t i = 0; i < na && n < LORA_SOIL_RECS_MAX; i++) {
        uint32_t hour_s = (uint32_t)(aggs[i].hour_ms / 1000u);
        uint8_t k = 0;                            // find/create the hour record
        for (; k < n; k++) if (recs[k].ts_hour_s == hour_s) break;
        if (k == n) {
            recs[n] = (lora_soil_rec_t){ .ts_hour_s = hour_s };
            n++;
        }
        if (aggs[i].kind == READING_SOIL_MOISTURE && aggs[i].depth_cm == 10) {
            recs[k].hs10 = aggs[i].mean; recs[k].has_hs10 = true;
        } else if (aggs[i].kind == READING_SOIL_MOISTURE && aggs[i].depth_cm == 30) {
            recs[k].hs30 = aggs[i].mean; recs[k].has_hs30 = true;
        } else if (aggs[i].kind == READING_AIR_TEMPERATURE) {
            recs[k].ta = aggs[i].mean; recs[k].has_ta = true;
        }
    }
    return n;
}

// Send one confirmed uplink (payload picked by state/mode) and apply any
// downlink. Captures the ACK's RSSI/SNR into the last-signal state, stamped with
// now_wall_ms. Assumes joined. Returns true if a downlink was decoded/stashed.
static bool do_uplink(const station_config_t *cfg, uint64_t now_wall_ms) {
    uint8_t payload[LORA_UPLINK_MAX];
    size_t plen = 0;
    lora_soil_rec_t soil[LORA_SOIL_RECS_MAX];
    uint8_t nsoil = 0;

    // Priority: pending CFG_ACK > dirty coords > mode payload. cfg==NULL (the
    // app's ping) always sends the plain forecast payload.
    if (s_ack_pending) {
        plen = lora_encode_uplink_cfg_ack(s_ack_applied, s_ack_rejected,
                                          payload, sizeof payload);
    } else if (cfg && cfg->has_coords &&
               (!s_coords_sent || s_coords_lat != cfg->lat_e7 || s_coords_lon != cfg->lon_e7)) {
        plen = lora_encode_uplink_coords(cfg->lat_e7, cfg->lon_e7,
                                         cfg->utc_offset_min, payload, sizeof payload);
    } else if (cfg && cfg->inference_mode == SAVIA_INFER_FORWARD &&
               (nsoil = build_soil_recs(now_wall_ms, soil)) > 0) {
        plen = lora_encode_uplink_soil(soil, nsoil, payload, sizeof payload);
    } else {
        // LOCAL (forecast min) or FORWARD with nothing new: keeps the RX window open.
        float hs30 = 0.0f;
        bool has = forecast_min_upcoming(now_wall_ms, &hs30);
        plen = lora_encode_uplink_forecast(has, hs30, payload, sizeof payload);
    }
    char tx_hex[2 * LORA_UPLINK_MAX + 1];
    bytes_to_hex(payload, plen, tx_hex);

    static const char *ok[]     = { "OK" };
    static const char *okfail[] = { "ERROR" };
    // Must hold AT+CMSGHEX="<2 hex chars per payload byte>" for the LARGEST frame:
    // a 48-byte buffer silently truncated any SOIL with >=2 records (>17 B), so
    // only the short forecast/cfg_ack frames ever reached TTN.
    char cmd[16 + 2 * LORA_UPLINK_MAX];
    _Static_assert(sizeof((char[16 + 2 * LORA_UPLINK_MAX]){0}) >= 14 + 2 * LORA_UPLINK_MAX + 1,
                   "AT command buffer must fit the largest hex payload");
    snprintf(cmd, sizeof cmd, "AT+PORT=%d", LORA_FPORT);
    at_exec(cmd, ok, 1, okfail, 1, 3000, NULL, 0);

    static const char *until[] = { "Done" };
    static const char *fail[]  = { "ERROR", "Please join", "Not join" };
    char rx_hex[2 * LORA_DOWNLINK_MAX + 1];
    s_signal_seen = false;                       // capture this uplink's ACK signal
    snprintf(cmd, sizeof cmd, "AT+CMSGHEX=\"%s\"", tx_hex);
    at_status_t st = at_exec(cmd, until, 1, fail, 3, LORA_UPLINK_TIMEOUT_MS,
                             rx_hex, sizeof rx_hex);
    if (s_signal_seen) {                          // the ACK reported RSSI/SNR
        s_has_signal = true;
        if (now_wall_ms) s_last_signal_ms = now_wall_ms;   // keep prior stamp if clock unset
        LOG_INFO("LoRa: signal RSSI %d dBm, SNR %d.%d dB\n",
                 s_rssi_dbm, s_snr_ddb / 10, (s_snr_ddb < 0 ? -s_snr_ddb : s_snr_ddb) % 10);
    }
    if (st == AT_FAIL) {
        LOG_WARN("LoRa: uplink rejected, will re-join\n");
        s_joined = false;
        return false;
    }
    if (st == AT_TIMEOUT) { LOG_WARN("LoRa: uplink timeout\n"); return false; }

    // Uplink went out: settle the send-once state for the payload we just sent.
    if (s_ack_pending) {
        s_ack_pending = false;
    } else if (cfg && cfg->has_coords &&
               (!s_coords_sent || s_coords_lat != cfg->lat_e7 || s_coords_lon != cfg->lon_e7)) {
        s_coords_sent = true;
        s_coords_lat = cfg->lat_e7;
        s_coords_lon = cfg->lon_e7;
    } else if (nsoil > 0) {
        for (uint8_t k = 0; k < nsoil; k++)
            if (soil[k].ts_hour_s > s_last_soil_hour_s) s_last_soil_hour_s = soil[k].ts_hour_s;
    }

    if (rx_hex[0] == '\0') { LOG_INFO("LoRa: uplink sent, no downlink\n"); return false; }

    uint8_t dl[LORA_DOWNLINK_MAX];
    int dn = hex_to_bytes(rx_hex, dl, sizeof dl);
    lora_downlink_t w;
    if (dn < 0 || !lora_decode_downlink(dl, (size_t) dn, &w)) {
        LOG_WARN("LoRa: bad downlink (%d B)\n", dn);
        return false;
    }

    if (w.type == LORA_DN_CONFIG) {
        // Stash the TLV for the supervisor (it owns cfg + persistence).
        s_cfg_tlv_len = w.tlv_len < sizeof s_cfg_tlv ? w.tlv_len : sizeof s_cfg_tlv;
        memcpy(s_cfg_tlv, w.tlv, s_cfg_tlv_len);
        LOG_INFO("LoRa downlink: config patch (%u B TLV)\n", (unsigned) s_cfg_tlv_len);
        return true;
    }

    uint32_t now_up = to_ms_since_boot(get_absolute_time());
    if (w.has_time) {
        uint64_t outage = 0;
        if (clock_apply_sync(w.time_ms, now_up, CLOCK_SRC_LORA, &outage)) {
            if (outage >= CLOCK_OUTAGE_WARN_MS)
                LOG_WARN("clock: board was powered off ~%llu min (LoRa sync)\n",
                         (unsigned long long) (outage / 60000ULL));
        } else {
            LOG_WARN("LoRa: implausible downlink clock (%llu), ignored\n",
                     (unsigned long long) w.time_ms);
        }
    }
    // A pure clock sync (both arrays empty) must NOT wipe the weather cache.
    if (w.n_past || w.n_future)
        weather_set(w.past_ta, w.n_past, w.future_ta, w.n_future, wall_now(now_up));
    LOG_INFO("LoRa downlink: %u past + %u future TA%s\n",
             w.n_past, w.n_future, w.has_time ? ", clock set" : "");
    return true;
}

// --- public API -------------------------------------------------------------

bool lora_init(const station_config_t *cfg) {
    if (!cfg || !cfg->lora_enabled) return false;
    lora_open(cfg->lora_uart_tx_gpio, cfg->lora_uart_rx_gpio);
    s_joined = s_ready && lora_join();
    s_attempted = false;
    LOG_INFO("LoRa: init joined=%d\n", s_joined);
    return s_joined;
}

bool lora_cycle(const station_config_t *cfg) {
    if (!s_ready || !cfg) {
        LOG_INFO("LoRa: cycle skip (ready=%d cfg=%d)\n", (int) s_ready, cfg != NULL);
        return false;
    }
    uint32_t up = to_ms_since_boot(get_absolute_time());

    // Clamp the app-set period so a bad/zero config can't spam the network.
    uint32_t period_s = cfg->lora_period_s;
    if (period_s < SAVIA_LORA_PERIOD_MIN_S) period_s = SAVIA_LORA_PERIOD_MIN_S;
    if (period_s > SAVIA_LORA_PERIOD_MAX_S) period_s = SAVIA_LORA_PERIOD_MAX_S;

    // One uplink per period; the first cycle after boot runs immediately.
    if (s_attempted && (up - s_last_attempt_ms) < period_s * 1000u) return false;
    s_attempted = true;
    s_last_attempt_ms = up;
    LOG_INFO("LoRa: cycle due (period=%us)\n", (unsigned) period_s);

    if (!s_joined) {                       // retry the OTAA join (e.g. after a drop)
        s_joined = lora_join();
        if (!s_joined) { LOG_WARN("LoRa: join failed, retry next period\n"); return false; }
        LOG_INFO("LoRa: joined network\n");
    }
    bool applied = do_uplink(cfg, wall_now(up));
    s_seq++;
    return applied;
}

uint32_t lora_secs_until_due(const station_config_t *cfg) {
    if (!s_ready || !cfg || !s_attempted) return 0;   // first cycle: due now
    uint32_t period_s = cfg->lora_period_s;
    if (period_s < SAVIA_LORA_PERIOD_MIN_S) period_s = SAVIA_LORA_PERIOD_MIN_S;
    if (period_s > SAVIA_LORA_PERIOD_MAX_S) period_s = SAVIA_LORA_PERIOD_MAX_S;
    uint32_t up = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = up - s_last_attempt_ms;
    uint32_t period_ms = period_s * 1000u;
    return elapsed >= period_ms ? 0 : (period_ms - elapsed + 999u) / 1000u;
}

bool lora_take_config_tlv(uint8_t *buf, size_t cap, size_t *len) {
    if (s_cfg_tlv_len == 0) return false;
    size_t n = s_cfg_tlv_len < cap ? s_cfg_tlv_len : cap;
    memcpy(buf, s_cfg_tlv, n);
    *len = n;
    s_cfg_tlv_len = 0;
    return true;
}

void lora_set_cfg_ack(uint8_t applied, uint8_t rejected) {
    s_ack_applied = applied;
    s_ack_rejected = rejected;
    s_ack_pending = true;
}

bool lora_ping(uint8_t tx_gpio, uint8_t rx_gpio, uint64_t now_wall_ms) {
    if (!s_ready || s_tx != tx_gpio || s_rx != rx_gpio) {   // first ping or pins changed
        if (!lora_open(tx_gpio, rx_gpio)) { s_seq++; return false; }  // module silent
        s_joined = false;
    }
    if (!s_joined) {
        s_joined = lora_join();
        if (!s_joined) { LOG_WARN("LoRa: ping join failed\n"); s_seq++; return false; }
        LOG_INFO("LoRa: ping joined network\n");
    }
    s_has_signal = false;             // this ping must measure its OWN downlink signal
    do_uplink(NULL, now_wall_ms);     // NULL cfg -> plain forecast payload
    s_seq++;                          // a fresh result is now visible in the status
    return s_joined;
}

void lora_get_status(lora_status_t *out) {
    if (!out) return;
    out->inited         = s_ready;
    out->joined         = s_joined;
    out->has_signal     = s_has_signal;
    out->rssi_dbm       = s_rssi_dbm;
    out->snr_ddb        = s_snr_ddb;
    // Double-read the 64-bit field: the BLE status read runs in the cyw43 async
    // context and can preempt the supervisor's two-word store.
    uint64_t a, b;
    do { a = s_last_signal_ms; b = s_last_signal_ms; } while (a != b);
    out->last_signal_ms = a;
    out->seq            = s_seq;
    strncpy(out->module, s_module, sizeof out->module - 1);
    out->module[sizeof out->module - 1] = '\0';
}

// Run a raw AT command and capture its reply lines (idle window between lines, a
// hard cap for slow commands like JOIN/CMSGHEX, and common terminators). Bumps seq
// LAST so the app's seq-gate only accepts a fully-written result.
void lora_at(uint8_t tx_gpio, uint8_t rx_gpio, const char *cmd) {
    if (!s_ready || s_tx != tx_gpio || s_rx != rx_gpio) lora_open(tx_gpio, rx_gpio);
    strncpy(s_at.cmd, cmd, sizeof s_at.cmd - 1);
    s_at.cmd[sizeof s_at.cmd - 1] = '\0';
    s_at.count = 0;

    if (s_ready) {
        uart_puts(s_uart, cmd);
        uart_puts(s_uart, "\r\n");
        LOG_DEBUG("LoRa AT -> %s\n", cmd);
        char line[LORA_AT_LINE_MAX];
        absolute_time_t hard = make_timeout_time_ms(13000);    // cap for slow cmds
        for (;;) {
            int64_t rem = absolute_time_diff_us(get_absolute_time(), hard);
            if (rem <= 0) break;
            uint32_t win = rem > 1500000 ? 1500u : (uint32_t)(rem / 1000);  // idle gap
            int n = read_line(line, sizeof line, make_timeout_time_ms(win));
            if (n <= 0) break;                                 // idle / hard cap -> done
            LOG_DEBUG("LoRa AT <- %s\n", line);
            if (s_at.count < LORA_AT_MAX_LINES) {
                strncpy(s_at.lines[s_at.count], line, LORA_AT_LINE_MAX - 1);
                s_at.lines[s_at.count][LORA_AT_LINE_MAX - 1] = '\0';
                s_at.count++;
            }
            if (strstr(line, "OK") || strstr(line, "ERROR") || strstr(line, "Done") ||
                strstr(line, "joined") || strstr(line, "failed")) break;   // terminators
        }
        if (s_at.count == 0) {
            strncpy(s_at.lines[0], "(sin respuesta)", LORA_AT_LINE_MAX - 1);
            s_at.lines[0][LORA_AT_LINE_MAX - 1] = '\0';
            s_at.count = 1;
        }
    } else {
        strncpy(s_at.lines[0], "(modulo sin respuesta)", LORA_AT_LINE_MAX - 1);
        s_at.lines[0][LORA_AT_LINE_MAX - 1] = '\0';
        s_at.count = 1;
    }
    s_at.seq++;                          // published last -> a complete result is visible
}

void lora_get_at_result(lora_at_result_t *out) {
    if (out) *out = s_at;
}

void lora_seed_last_signal(int16_t rssi_dbm, int16_t snr_ddb, uint64_t last_signal_ms) {
    if (last_signal_ms == 0) return;             // nothing persisted yet
    s_rssi_dbm = rssi_dbm;
    s_snr_ddb = snr_ddb;
    s_last_signal_ms = last_signal_ms;
    s_has_signal = true;
}
