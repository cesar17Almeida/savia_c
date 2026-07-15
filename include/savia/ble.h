// BLE GATT peripheral via BTstack on the CYW43439 (same chip on Pico WH and
// Pico 2 W, so this layer is identical on both). Serves the savia_py contract.
#ifndef SAVIA_BLE_H
#define SAVIA_BLE_H

#include "savia/config.h"
#include <stddef.h>

// cfg is held (not copied): config writes from the app mutate it in place, so
// the next supervisor cycle picks up a new sleep time.
void ble_init(station_config_t *cfg);

// Service the BTstack run loop for up to `budget_ms`, then return so the
// supervisor can go back to sleep. Called during the daily BLE window.
void ble_poll(uint32_t budget_ms);

// True once (then clears) if the app changed config over BLE; the supervisor
// loop uses it to persist cfg to flash outside the BLE callback context.
bool ble_take_config_dirty(void);

// True once (then clears) if the app requested an on-demand LoRa ping; the
// supervisor runs it (blocking AT) outside the BLE callback context.
bool ble_take_lora_ping(void);
// True once (then clears) if the app asked the station to run inference now.
bool ble_take_infer_trigger(void);
// Non-destructive peeks so the light-sleep nap can end early to service a request.
bool ble_lora_ping_pending(void);
bool ble_infer_pending(void);

// AT terminal: take the queued raw AT command (copies it into cmd, returns true if
// one was queued). The supervisor runs it on the module outside the BLE callback.
bool ble_take_lora_at(char *cmd, size_t cap);
bool ble_lora_at_pending(void);

// SDI-12 console: take the queued raw probe command + its data GPIO. The
// supervisor runs the (blocking) transaction outside the BLE callback context.
bool ble_take_sdi12(char *cmd, size_t cap, uint8_t *gpio);
bool ble_sdi12_pending(void);

// Actuator: take the queued {port, on} request. The supervisor validates the
// slot type and drives the GPIO (only SENSOR_ACTUATOR_DIGITAL switches).
bool ble_take_act(uint8_t *port, bool *on);
bool ble_act_pending(void);

// Power the radio down / back up around a real deep-sleep interval. suspend()
// stops advertising and deinits the CYW43 (the dominant consumer); resume()
// brings the stack + advertising back. No-ops when BLE is compiled out.
void ble_radio_suspend(void);
void ble_radio_resume(void);

// State for the status LED. Both false when BLE is compiled out.
bool ble_is_connected(void);     // a central is connected (paired)
bool ble_is_advertising(void);   // radio up and advertising (discoverable)

#endif // SAVIA_BLE_H
