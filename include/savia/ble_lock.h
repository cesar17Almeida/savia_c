// Lock shared with the BLE stack (the cyw43 async_context): BTstack callbacks run
// under it, so the supervisor takes it around state both sides mutate. No-op
// without BLE. Only valid once ble_init() has brought the radio up.
#ifndef SAVIA_BLE_LOCK_H
#define SAVIA_BLE_LOCK_H

#if SAVIA_ENABLE_BLE
#include "pico/cyw43_arch.h"
static inline void savia_ble_lock(void)   { cyw43_arch_lwip_begin(); }
static inline void savia_ble_unlock(void) { cyw43_arch_lwip_end(); }
#else
static inline void savia_ble_lock(void)   {}
static inline void savia_ble_unlock(void) {}
#endif

#endif // SAVIA_BLE_LOCK_H
