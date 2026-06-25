#include "savia/sensor.h"
#include "pico/stdlib.h"

// Sensor acquisition dispatcher. Each configured slot is read by its type:
//   - SDI-12 (AquaCheck / generic): 1200 baud 7e1 inverted on a single 5 V
//     half-duplex line with a 12 ms break; the RP2040/RP2350 PIO does the timing
//     (the Pi needed an Arduino bridge for it). A 3.3<->5 V level shifter sits in
//     between. AquaCheck has a fixed value layout; a generic SDI-12 sensor's values
//     are labelled by the installer (slot->map.sdi12.ch[]).
//   - analog linear: read an ADC channel (GP26..28) and apply scale*raw + offset.
//   - 1-Wire DS18B20: reset + ROM match + Convert T, read scratchpad -> degrees C.
// The real peripheral I/O is still TODO(hw); until then mock_enabled drives the
// pipeline and these return plausible placeholders so acquire->store is exercisable.

void sensor_init(const station_config_t *cfg) {
    (void)cfg;
    // TODO(hw): claim a PIO state machine per SDI-12/1-Wire GPIO and load its
    // program; configure the ADC for each analog slot's channel.
}

static int measure_sdi12(const savia_sensor_slot_t *slot, savia_reading_t *out, int max) {
    (void)slot;
    if (max < 2) return 0;
    // TODO(hw): "<addr>M!" -> "atttn" -> wait ttt s -> "<addr>D0!"/"D1!"... collect n
    //   VWC values; map value[i] -> depth (AquaCheck fixed; generic via
    //   slot->map.sdi12.ch[i]). "<addr>M1!" for soil temperature. Clamp VWC to [0,1].
    uint64_t now = to_ms_since_boot(get_absolute_time());
    out[0] = (savia_reading_t){ now, 1, 10, READING_SOIL_MOISTURE, 0.75f };
    out[1] = (savia_reading_t){ now, 1, 30, READING_SOIL_MOISTURE, 0.77f };
    return 2;
}

static int measure_analog(const savia_sensor_slot_t *slot, savia_reading_t *out, int max) {
    if (max < 1) return 0;
    // TODO(hw): adc_select_input(slot->gpio - 26); raw = adc_read() / 4095.0f.
    float raw = 0.5f;   // placeholder normalized reading
    float value = slot->map.analog.scale * raw + slot->map.analog.offset;
    uint64_t now = to_ms_since_boot(get_absolute_time());
    out[0] = (savia_reading_t){ now, 1, slot->depth_cm, slot->kind, value };
    return 1;
}

static int measure_ds18b20(const savia_sensor_slot_t *slot, savia_reading_t *out, int max) {
    if (max < 1) return 0;
    // TODO(hw): 1-Wire reset + ROM match, Convert T (0x44), wait, read scratchpad -> degC.
    uint64_t now = to_ms_since_boot(get_absolute_time());
    out[0] = (savia_reading_t){ now, 1, slot->depth_cm, slot->kind, 21.5f };
    return 1;
}

// Measure one configured slot. Returns the number of readings written, 0 if none.
int sensor_measure(const savia_sensor_slot_t *slot, savia_reading_t *out, int max) {
    switch (slot->type) {
        case SENSOR_SDI12_AQUACHECK:
        case SENSOR_SDI12_GENERIC:   return measure_sdi12(slot, out, max);
        case SENSOR_ANALOG_LINEAR:   return measure_analog(slot, out, max);
        case SENSOR_ONEWIRE_DS18B20: return measure_ds18b20(slot, out, max);
        default:                     return 0;   // SENSOR_NONE
    }
}
