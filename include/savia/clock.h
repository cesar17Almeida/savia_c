// Software wall clock. The Pico has no battery-backed RTC, so the app (or LoRa
// downlink) sets the time via time_sync; we keep an epoch base relative to the
// board uptime. Pure logic (uptime passed in) so it unit-tests on the host.
#ifndef SAVIA_CLOCK_H
#define SAVIA_CLOCK_H

#include <stdint.h>
#include <stdbool.h>

// Set the wall clock: `epoch_ms` is "now", `uptime_ms` the current ms-since-boot.
void clock_set(uint64_t epoch_ms, uint64_t uptime_ms);

// Current epoch ms, given the current uptime. Returns 0 if never synced.
uint64_t clock_now(uint64_t uptime_ms);

// True once time_sync has been applied at least once.
bool clock_is_set(void);

// Epoch ms of the last sync (0 if never). For the status payload.
uint64_t clock_last_sync_ms(void);

#endif // SAVIA_CLOCK_H
