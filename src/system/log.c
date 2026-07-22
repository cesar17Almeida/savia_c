// Logging sink: prints to serial (as before) AND keeps the last N lines in an
// in-RAM ring so the app can pull them over BLE (data_request kind="logs").
#include "savia/log.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define LOG_RING_LINES 24
#define LOG_LINE_LEN   80

static char     s_ring[LOG_RING_LINES][LOG_LINE_LEN];
static unsigned s_head;     // next slot to write
static unsigned s_count;    // lines retained (<= LOG_RING_LINES)

int savia_log_level = SAVIA_LOG_LEVEL;   // runtime level, seeded from the build default
void savia_log_set_level(int level) { savia_log_level = level; }

// Optional clock source (set from main). Without it, lines carry no timestamp.
static uint64_t (*s_clock)(bool *wall) = NULL;
void savia_log_set_clock(uint64_t (*now_ms)(bool *wall)) { s_clock = now_ms; }

static void (*s_flush)(void) = NULL;
void savia_log_set_flush(void (*flush)(void)) { s_flush = flush; }

// Render the per-line timestamp prefix into `out`; returns its length (0 if no
// clock). HH:MM:SS once the wall clock is synced, else +Ns since boot.
static int stamp_prefix(char *out, size_t cap) {
    if (!s_clock) return 0;
    bool wall = false;
    uint64_t ms = s_clock(&wall);
    int n;
    if (wall) {
        uint64_t s = ms / 1000;
        n = snprintf(out, cap, "%02u:%02u:%02u ",
                     (unsigned)((s / 3600) % 24), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
    } else {
        n = snprintf(out, cap, "+%lus ", (unsigned long)(ms / 1000));
    }
    return n > 0 ? n : 0;
}

static void ring_push(const char *line) {
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) n--;  // strip EOL
    char *dst = s_ring[s_head];

    char ts[16];
    int tn = stamp_prefix(ts, sizeof(ts));
    size_t pos = 0;
    if (tn > 0 && (size_t) tn < LOG_LINE_LEN - 1) {
        memcpy(dst, ts, (size_t) tn);
        pos = (size_t) tn;
    }
    size_t avail = LOG_LINE_LEN - 1 - pos;
    if (n > avail) n = avail;
    memcpy(dst + pos, line, n);
    dst[pos + n] = '\0';

    s_head = (s_head + 1) % LOG_RING_LINES;
    if (s_count < LOG_RING_LINES) s_count++;
}

void savia_log_write(const char *fmt, ...) {
    char buf[LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("%s", buf);   // serial output, unchanged
    if (s_flush) s_flush();
    ring_push(buf);
}

void savia_log_hexdump(const char *label, const uint8_t *buf, unsigned len) {
    char line[LOG_LINE_LEN];
    int off = snprintf(line, sizeof(line), "%s [%u B]:", label, len);
    for (unsigned i = 0; i < len && off > 0 && off < (int) sizeof(line) - 4; i++)
        off += snprintf(line + off, sizeof(line) - (size_t) off, " %02x", buf[i]);
    printf("%s\n", line);
    if (s_flush) s_flush();
    ring_push(line);
}

unsigned savia_log_count(void) { return s_count; }

const char *savia_log_line(unsigned i) {
    if (i >= s_count) return "";
    unsigned start = (s_head + LOG_RING_LINES - s_count) % LOG_RING_LINES;  // oldest
    return s_ring[(start + i) % LOG_RING_LINES];
}
