// Tiny leveled logging over stdio. Level is a compile-time switch:
//   -DSAVIA_LOG_LEVEL=0  -> DEBUG (verbose: + raw byte dumps of BLE writes)
//   -DSAVIA_LOG_LEVEL=1  -> INFO  (default: events only)
//   -DSAVIA_LOG_LEVEL=2  -> WARN  (quiet)
#ifndef SAVIA_LOG_H
#define SAVIA_LOG_H

#include <stdio.h>
#include <stdint.h>

#define SAVIA_LOG_DEBUG 0
#define SAVIA_LOG_INFO  1
#define SAVIA_LOG_WARN  2

#ifndef SAVIA_LOG_LEVEL
#define SAVIA_LOG_LEVEL SAVIA_LOG_INFO
#endif

#define LOG_DEBUG(...) do { if (SAVIA_LOG_LEVEL <= SAVIA_LOG_DEBUG) printf(__VA_ARGS__); } while (0)
#define LOG_INFO(...)  do { if (SAVIA_LOG_LEVEL <= SAVIA_LOG_INFO)  printf(__VA_ARGS__); } while (0)
#define LOG_WARN(...)  do { if (SAVIA_LOG_LEVEL <= SAVIA_LOG_WARN)  printf(__VA_ARGS__); } while (0)

// Hex dump of a buffer at DEBUG level (e.g. the raw bytes the phone wrote).
static inline void log_hexdump(const char *label, const uint8_t *buf, unsigned len) {
    if (SAVIA_LOG_LEVEL > SAVIA_LOG_DEBUG) return;
    printf("%s [%u B]:", label, len);
    for (unsigned i = 0; i < len; i++) printf(" %02x", buf[i]);
    printf("\n");
}

#endif // SAVIA_LOG_H
