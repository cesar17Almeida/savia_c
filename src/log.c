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

static void ring_push(const char *line) {
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) n--;  // strip EOL
    if (n > LOG_LINE_LEN - 1) n = LOG_LINE_LEN - 1;
    char *dst = s_ring[s_head];
    memcpy(dst, line, n);
    dst[n] = '\0';
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
    ring_push(buf);
}

void savia_log_hexdump(const char *label, const uint8_t *buf, unsigned len) {
    char line[LOG_LINE_LEN];
    int off = snprintf(line, sizeof(line), "%s [%u B]:", label, len);
    for (unsigned i = 0; i < len && off > 0 && off < (int) sizeof(line) - 4; i++)
        off += snprintf(line + off, sizeof(line) - (size_t) off, " %02x", buf[i]);
    printf("%s\n", line);
    ring_push(line);
}

unsigned savia_log_count(void) { return s_count; }

const char *savia_log_line(unsigned i) {
    if (i >= s_count) return "";
    unsigned start = (s_head + LOG_RING_LINES - s_count) % LOG_RING_LINES;  // oldest
    return s_ring[(start + i) % LOG_RING_LINES];
}
