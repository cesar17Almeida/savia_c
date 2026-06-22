#include "savia/sensor.h"
#include "pico/stdlib.h"

// SDI-12 over the RP2040/RP2350 PIO. SDI-12 is 1200 baud, 7e1, inverted, on a
// single 5 V half-duplex line with a 12 ms break to wake the sensor -- the PIO
// handles that timing in real time (the Pi needed an Arduino bridge for it).
// A 3.3<->5 V level shifter sits between the GPIO and the SDI-12 data line.

void sensor_init(const station_config_t *cfg) {
    (void)cfg;
    // TODO(hw): claim a PIO state machine per configured sensor GPIO and load
    // the SDI-12 program (break + 1200-7e1-inverted framing).
}

int sensor_measure(const savia_sensor_slot_t *slot, savia_reading_t *out, int max) {
    if (slot->type == SENSOR_NONE || max < 2) {
        return 0;
    }
    // TODO(hw): run the SDI-12 transaction (same sequence as savia_py):
    //   "<addr>M!"  -> parse "atttn" -> wait ttt s -> "<addr>D0!"/"D1!"...
    //                  collect n VWC values, map value[i] -> depths_cm[i]
    //   "<addr>M1!" -> soil temperature, same drain
    // then clamp moisture to [0,1].
    //
    // Placeholder readings so the acquire->store pipeline is exercisable.
    uint64_t now = to_ms_since_boot(get_absolute_time());
    out[0] = (savia_reading_t){ now, 1, 10, READING_SOIL_MOISTURE, 0.75f };
    out[1] = (savia_reading_t){ now, 1, 30, READING_SOIL_MOISTURE, 0.77f };
    return 2;
}
