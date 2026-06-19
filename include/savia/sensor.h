// Sensor acquisition. Today: AquaCheck over SDI-12 (the RP2040/RP2350 can drive
// the SDI-12 timing directly via PIO + a 3.3<->5 V level shifter, absorbing the
// Arduino bridge used on the Pi). The interface is sensor-type agnostic so new
// sensor kinds slot in behind the same measure() call.
#ifndef SAVIA_SENSOR_H
#define SAVIA_SENSOR_H

#include "savia/config.h"
#include "savia/types.h"

void sensor_init(const station_config_t *cfg);

// Measure one configured slot. Writes up to `max` readings into `out`.
// Returns the number written, or <0 on a sensor/transaction error.
int sensor_measure(const savia_sensor_slot_t *slot, savia_reading_t *out, int max);

#endif // SAVIA_SENSOR_H
