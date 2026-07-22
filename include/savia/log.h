// Tiny leveled logging over stdio. Level is a compile-time switch:
//   -DSAVIA_LOG_LEVEL=0  -> DEBUG (verbose: + raw byte dumps of BLE writes)
//   -DSAVIA_LOG_LEVEL=1  -> INFO  (default: events only)
//   -DSAVIA_LOG_LEVEL=2  -> WARN  (quiet)
// Lines also land in an in-RAM ring (log.c) so the app can read them over BLE.
#ifndef SAVIA_LOG_H
#define SAVIA_LOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAVIA_LOG_DEBUG 0
#define SAVIA_LOG_INFO  1
#define SAVIA_LOG_WARN  2

#ifndef SAVIA_LOG_LEVEL
#define SAVIA_LOG_LEVEL SAVIA_LOG_INFO   // startup default
#endif

// Runtime level (settable over BLE from the app). Macros gate on this, so a
// build keeps every level compiled in and the level can switch info<->debug live.
extern int savia_log_level;
void savia_log_set_level(int level);

// Implemented in log.c: printf to serial + append to the ring buffer.
void savia_log_write(const char *fmt, ...);
void savia_log_hexdump(const char *label, const uint8_t *buf, unsigned len);

// Optional time source for log line timestamps, wired from main once the clock
// + SDK uptime exist (log.c stays SDK-free without it). now_ms() returns ms;
// *wall=true => epoch ms (shown HH:MM:SS UTC), false => ms-since-boot (shown +Ns).
void savia_log_set_clock(uint64_t (*now_ms)(bool *wall));
// Optional post-line flush (Pico: stdio_flush) so USB-CDC never drops lines.
void savia_log_set_flush(void (*flush)(void));

#define LOG_DEBUG(...) do { if (savia_log_level <= SAVIA_LOG_DEBUG) savia_log_write(__VA_ARGS__); } while (0)
#define LOG_INFO(...)  do { if (savia_log_level <= SAVIA_LOG_INFO)  savia_log_write(__VA_ARGS__); } while (0)
#define LOG_WARN(...)  do { if (savia_log_level <= SAVIA_LOG_WARN)  savia_log_write(__VA_ARGS__); } while (0)

// Hex dump of a buffer at DEBUG level (e.g. the raw bytes the phone wrote).
#define log_hexdump(label, buf, len) \
    do { if (savia_log_level <= SAVIA_LOG_DEBUG) savia_log_hexdump(label, buf, len); } while (0)

// Ring accessors for the BLE log channel (data_request kind="logs").
unsigned    savia_log_count(void);          // lines currently retained
const char *savia_log_line(unsigned i);     // i: 0 = oldest retained .. count-1 = newest

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // SAVIA_LOG_H
