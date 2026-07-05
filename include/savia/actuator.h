// Digital-actuator state (valve/relay slots, SENSOR_ACTUATOR_DIGITAL). Pure
// state only (SDK-free, host-testable): the supervisor drives the real GPIO and
// calls set(); the BLE status read serves the mask. Everything is OFF at boot by
// design (a reboot must never leave a valve open).
#ifndef SAVIA_ACTUATOR_H
#define SAVIA_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>

// Set/get the ON state for a slot port (1..SAVIA_MAX_SENSORS).
void actuator_set(uint8_t port, bool on);
bool actuator_is_on(uint8_t port);

#endif // SAVIA_ACTUATOR_H
