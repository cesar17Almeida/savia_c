// BLE GATT peripheral via BTstack on the CYW43439 (same chip on Pico WH and
// Pico 2 W, so this layer is identical on both). Serves the savia_py contract.
#ifndef SAVIA_BLE_H
#define SAVIA_BLE_H

#include "savia/config.h"

void ble_init(const station_config_t *cfg);

// Service the BTstack run loop for up to `budget_ms`, then return so the
// supervisor can go back to sleep. Called during the daily BLE window.
void ble_poll(uint32_t budget_ms);

#endif // SAVIA_BLE_H
