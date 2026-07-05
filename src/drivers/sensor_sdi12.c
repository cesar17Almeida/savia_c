#include "savia/sensor.h"
#include "savia/sdi12.h"
#include "savia/log.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <string.h>
#include <stdio.h>

// Sensor acquisition dispatcher. Each configured slot is read by its type:
//   - SDI-12 (AquaCheck / generic): bit-banged 1200 7E1 INVERTED on one wire,
//     oversampled RX decode -- ported from tools/sdi12_bringup (validated on the
//     real probe 2026-06-27, see AQUACHECK_RESPONSES.md). 3.3 V direct, no shifter.
//   - DHT11: proprietary single-wire us protocol, bit-banged.
//   - HC-SR04: 10 us trigger pulse on gpio, echo width on gpio2 -> distance mm.
//   - analog linear: ADC read (GP26..28), value = scale*raw01 + offset.
//   - 1-Wire DS18B20: reset/presence + skip ROM + convert + scratchpad -> degC.
//   - actuator: output-only, driven by the supervisor (no measure).

// --- SDI-12 bit-bang (from tools/sdi12_bringup, parameterized by gpio) --------

#define SDI_BIT_US   833
#define SDI_OS_US    104          // RX sample period (~8x oversampling)
#define SDI_OS_BIT   8            // samples per bit
#define SDI_OS_MAX   6000         // ~624 ms capture window
static uint8_t s_os[SDI_OS_MAX];  // oversample buffer (static: off the stack)

static void sdi_tx_char(uint8_t gpio, char ch) {
    uint8_t d = (uint8_t) ch & 0x7F; int ones = 0;
    gpio_put(gpio, 1); busy_wait_us(SDI_BIT_US);                  // start (spacing=HIGH)
    for (int i = 0; i < 7; i++) {
        int bit = (d >> i) & 1; if (bit) ones++;
        gpio_put(gpio, bit ? 0 : 1); busy_wait_us(SDI_BIT_US);    // logic1 = LOW
    }
    gpio_put(gpio, (ones & 1) ? 0 : 1); busy_wait_us(SDI_BIT_US); // even parity
    gpio_put(gpio, 0); busy_wait_us(SDI_BIT_US);                  // stop (marking=LOW)
}

static void sdi_send(uint8_t gpio, const char *s) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1); busy_wait_us(13000);   // break >= 12 ms
    gpio_put(gpio, 0); busy_wait_us(9000);    // marking >= 8.33 ms
    while (*s) sdi_tx_char(gpio, *s++);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_down(gpio);
}

static int sdi_capture(uint8_t gpio, uint32_t window_ms) {
    int total = (int)((uint32_t) window_ms * 1000u / SDI_OS_US);
    if (total > SDI_OS_MAX) total = SDI_OS_MAX;
    absolute_time_t t = get_absolute_time();
    for (int i = 0; i < total; i++) {
        s_os[i] = (uint8_t) gpio_get(gpio);
        t = delayed_by_us(t, SDI_OS_US);
        busy_wait_until(t);                   // fixed grid -> no cumulative drift
    }
    return total;
}

static int sdi_decode(int ns, char *out, int max) {
    int n = 0, i = 0;
    while (n < max - 1) {
        while (i < ns && s_os[i] != 0) i++;   // skip HIGH
        while (i < ns && s_os[i] == 0) i++;   // skip LOW idle -> first HIGH = start
        if (i >= ns) break;
        int start = i;
        uint8_t c = 0; int bad = 0;
        for (int b = 0; b < 7; b++) {
            int idx = start + SDI_OS_BIT * b + SDI_OS_BIT + SDI_OS_BIT / 2;
            if (idx >= ns) { bad = 1; break; }
            if (s_os[idx] == 0) c |= (uint8_t)(1u << b);   // LOW = logic 1
        }
        if (bad) break;
        char ch = (char)(c & 0x7F);
        if (ch != '\r' && ch != '\n') out[n++] = ch;   // strip CRLF for parsing
        i = start + 10 * SDI_OS_BIT - SDI_OS_BIT / 2;
    }
    out[n] = '\0';
    return n;
}

// One raw command -> decoded printable reply (CRLF stripped). Blocking (~win ms).
static int sdi12_transact(uint8_t gpio, const char *cmd, char *reply, size_t cap,
                          uint32_t win_ms) {
    if (cap < 2) return 0;
    sdi_send(gpio, cmd);
    int ns = sdi_capture(gpio, win_ms);
    return sdi_decode(ns, reply, (int) cap);
}

// --- SDI-12 console (BLE op "sdi12"; mirrors the LoRa AT terminal) -----------

static sdi12_console_result_t s_console;

void sdi12_console_run(uint8_t gpio, const char *cmd) {
    strncpy(s_console.cmd, cmd, sizeof s_console.cmd - 1);
    s_console.cmd[sizeof s_console.cmd - 1] = '\0';
    char reply[SDI12_LINE_MAX];
    int n = sdi12_transact(gpio, cmd, reply, sizeof reply, 800);
    strncpy(s_console.lines[0], n > 0 ? reply : "(sin respuesta)", SDI12_LINE_MAX - 1);
    s_console.lines[0][SDI12_LINE_MAX - 1] = '\0';
    s_console.count = 1;
    s_console.seq++;             // published last -> a complete result is visible
}

void sdi12_get_console_result(sdi12_console_result_t *out) {
    if (out) *out = s_console;
}

// --- measurement drivers ------------------------------------------------------

void sensor_init(const station_config_t *cfg) {
    adc_init();
    if (!cfg) return;
    uint8_t n = cfg->sensor_count <= SAVIA_MAX_SENSORS ? cfg->sensor_count : SAVIA_MAX_SENSORS;
    for (uint8_t i = 0; i < n; i++) {
        const savia_sensor_slot_t *s = &cfg->sensors[i];
        if (s->type == SENSOR_ANALOG_LINEAR && s->gpio >= 26 && s->gpio <= 28)
            adc_gpio_init(s->gpio);
    }
}

// Full SDI-12 measurement: aM! -> "atttn" -> wait ttt s -> aD0..Dk until n values.
static int sdi12_measure_values(uint8_t gpio, char addr, float *vals, int max) {
    char cmd[8], reply[SDI12_LINE_MAX];
    snprintf(cmd, sizeof cmd, "%cM!", addr);
    if (sdi12_transact(gpio, cmd, reply, sizeof reply, 500) < 5) return -1;
    int delay_s = 0, nvals = 0;
    if (!sdi12_parse_measure_hdr(reply, &delay_s, &nvals)) return -1;
    if (nvals > max) nvals = max;
    if (delay_s > 0) sleep_ms((uint32_t) delay_s * 1000u + 200u);

    int got = 0;
    for (int d = 0; d <= 9 && got < nvals; d++) {
        snprintf(cmd, sizeof cmd, "%cD%d!", addr, d);
        if (sdi12_transact(gpio, cmd, reply, sizeof reply, 600) <= 1) break;
        int before = got;
        got = sdi12_parse_values(reply, vals, got, nvals);
        if (got == before) break;             // empty D reply -> probe is done
    }
    return got;
}

static int measure_sdi12(const savia_sensor_slot_t *slot,
                         savia_reading_t *out, int max) {
    float vals[8];
    int n = sdi12_measure_values(slot->gpio, slot->address ? slot->address : '0',
                                 vals, 8);
    if (n <= 0) { LOG_WARN("sdi12: no reply on GP%u\n", slot->gpio); return -1; }

    int w = 0;
    if (slot->type == SENSOR_SDI12_AQUACHECK) {
        // Probe 1120-0404: 4 sensors top->bottom at 10/20/30/40 cm (immersion-
        // verified). Values are SFU (~0..120); a true VWC needs a soil calibration
        // -- until then store SFU/100 clamped to [0,1] (documented approximation).
        static const uint8_t depths[4] = { 10, 20, 30, 40 };
        for (int i = 0; i < n && i < 4 && w < max; i++) {
            float vwc = vals[i] / 100.0f;
            if (vwc < 0.0f) vwc = 0.0f;
            if (vwc > 1.0f) vwc = 1.0f;
            out[w++] = (savia_reading_t){ 0, 1, depths[i], READING_SOIL_MOISTURE, vwc };
        }
    } else {
        // Generic SDI-12: the installer labelled each value index -> {kind, depth}.
        uint8_t cc = slot->map.sdi12.count <= SAVIA_SDI12_MAX_CHANNELS
                   ? slot->map.sdi12.count : SAVIA_SDI12_MAX_CHANNELS;
        for (int i = 0; i < n && i < cc && w < max; i++) {
            out[w++] = (savia_reading_t){ 0, 1, slot->map.sdi12.ch[i].depth_cm,
                                          slot->map.sdi12.ch[i].kind, vals[i] };
        }
    }
    return w;
}

static int measure_analog(const savia_sensor_slot_t *slot,
                          savia_reading_t *out, int max) {
    if (max < 1 || slot->gpio < 26 || slot->gpio > 28) return 0;
    adc_select_input(slot->gpio - 26);
    float raw01 = (float) adc_read() / 4095.0f;
    float value = slot->map.analog.scale * raw01 + slot->map.analog.offset;
    out[0] = (savia_reading_t){ 0, 1, slot->depth_cm, slot->kind, value };
    return 1;
}

// --- DHT11: proprietary single-wire protocol. IRQs stay on: the generous
// timeouts tolerate small preemptions and a corrupted frame fails the checksum. --

static int wait_level(uint8_t gpio, bool level, uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (gpio_get(gpio) != level) {
        if (time_us_32() - t0 > timeout_us) return -1;
    }
    return (int)(time_us_32() - t0);
}

static int measure_dht11(const savia_sensor_slot_t *slot,
                         savia_reading_t *out, int max) {
    if (max < 2) return 0;
    uint8_t g = slot->gpio;
    gpio_init(g);
    gpio_set_dir(g, GPIO_OUT);
    gpio_put(g, 0); sleep_ms(20);            // >18 ms start signal
    gpio_set_dir(g, GPIO_IN);
    gpio_pull_up(g);

    // Handshake: sensor answers LOW ~80 us then HIGH ~80 us, then bits start.
    if (wait_level(g, 0, 200) < 0) return -1;
    if (wait_level(g, 1, 200) < 0) return -1;
    if (wait_level(g, 0, 200) < 0) return -1;

    // 40 bits: each = ~50 us LOW then HIGH of ~27 us (0) or ~70 us (1).
    uint8_t data[5] = { 0 };
    for (int b = 0; b < 40; b++) {
        if (wait_level(g, 1, 100) < 0) return -1;
        int high_us = wait_level(g, 0, 150);
        if (high_us < 0) return -1;
        if (high_us > 45) data[b / 8] |= (uint8_t)(1u << (7 - b % 8));
    }
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        LOG_WARN("dht11: checksum mismatch on GP%u\n", g);
        return -1;
    }
    // DHT11 integer resolution: data[0] = %RH, data[2] = degC.
    out[0] = (savia_reading_t){ 0, 1, 0, READING_AIR_HUMIDITY, (float) data[0] };
    out[1] = (savia_reading_t){ 0, 1, 0, READING_AIR_TEMPERATURE, (float) data[2] };
    return 2;
}

static int measure_hcsr04(const savia_sensor_slot_t *slot,
                          savia_reading_t *out, int max) {
    if (max < 1 || slot->gpio2 == SAVIA_GPIO_NONE) return 0;
    uint8_t trig = slot->gpio, echo = slot->gpio2;
    gpio_init(trig); gpio_set_dir(trig, GPIO_OUT); gpio_put(trig, 0);
    gpio_init(echo); gpio_set_dir(echo, GPIO_IN);
    busy_wait_us(5);
    gpio_put(trig, 1); busy_wait_us(10); gpio_put(trig, 0);   // 10 us trigger

    if (wait_level(echo, 1, 30000) < 0) return -1;            // echo start
    uint32_t t0 = time_us_32();
    if (wait_level(echo, 0, 30000) < 0) return -1;            // echo end (<= ~5 m)
    uint32_t us = time_us_32() - t0;
    float mm = (float) us * 0.1715f;                          // us * 343 m/s / 2
    out[0] = (savia_reading_t){ 0, 1, 0,
                                slot->kind ? slot->kind : READING_DISTANCE, mm };
    return 1;
}

// --- 1-Wire DS18B20 ------------------------------------------------------------

static bool ow_reset(uint8_t g) {
    gpio_init(g); gpio_set_dir(g, GPIO_OUT); gpio_put(g, 0);
    busy_wait_us(480);
    gpio_set_dir(g, GPIO_IN); gpio_pull_up(g);
    busy_wait_us(70);
    bool presence = !gpio_get(g);
    busy_wait_us(410);
    return presence;
}

static void ow_write_bit(uint8_t g, int bit) {
    gpio_set_dir(g, GPIO_OUT); gpio_put(g, 0);
    busy_wait_us(bit ? 6 : 60);
    gpio_set_dir(g, GPIO_IN);                 // release (pull-up raises the line)
    busy_wait_us(bit ? 64 : 10);
}

static int ow_read_bit(uint8_t g) {
    gpio_set_dir(g, GPIO_OUT); gpio_put(g, 0);
    busy_wait_us(6);
    gpio_set_dir(g, GPIO_IN);
    busy_wait_us(9);
    int bit = gpio_get(g);
    busy_wait_us(55);
    return bit;
}

static void ow_write_byte(uint8_t g, uint8_t b) {
    for (int i = 0; i < 8; i++) ow_write_bit(g, (b >> i) & 1);
}

static uint8_t ow_read_byte(uint8_t g) {
    uint8_t b = 0;
    for (int i = 0; i < 8; i++) if (ow_read_bit(g)) b |= (uint8_t)(1u << i);
    return b;
}

static int measure_ds18b20(const savia_sensor_slot_t *slot,
                           savia_reading_t *out, int max) {
    if (max < 1) return 0;
    uint8_t g = slot->gpio;
    if (!ow_reset(g)) { LOG_WARN("ds18b20: no presence on GP%u\n", g); return -1; }
    ow_write_byte(g, 0xCC);                   // skip ROM (single drop)
    ow_write_byte(g, 0x44);                   // convert T
    sleep_ms(750);                            // max conversion @ 12 bit
    if (!ow_reset(g)) return -1;
    ow_write_byte(g, 0xCC);
    ow_write_byte(g, 0xBE);                   // read scratchpad
    uint8_t lo = ow_read_byte(g), hi = ow_read_byte(g);
    int16_t raw = (int16_t)((hi << 8) | lo);
    float degc = (float) raw / 16.0f;
    out[0] = (savia_reading_t){ 0, 1, slot->depth_cm,
                                slot->kind ? slot->kind : READING_AIR_TEMPERATURE, degc };
    return 1;
}

// Measure one configured slot. Returns readings written, 0 if none, <0 on error.
// Callers stamp .ts_ms; .port here is 1 and main.c rewrites it per slot index.
int sensor_measure(const savia_sensor_slot_t *slot, savia_reading_t *out, int max) {
    switch (slot->type) {
        case SENSOR_SDI12_AQUACHECK:
        case SENSOR_SDI12_GENERIC:    return measure_sdi12(slot, out, max);
        case SENSOR_ANALOG_LINEAR:    return measure_analog(slot, out, max);
        case SENSOR_ONEWIRE_DS18B20:  return measure_ds18b20(slot, out, max);
        case SENSOR_DHT11:            return measure_dht11(slot, out, max);
        case SENSOR_HCSR04:           return measure_hcsr04(slot, out, max);
        default:                      return 0;   // SENSOR_NONE / actuator (output-only)
    }
}
