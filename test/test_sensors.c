// Host test for the sensor configuration path: build a config patch carrying a
// sensors[] table (all four molds), parse it (ble_parse_config_patch), validate it
// atomically against the pin inventory (pinmap_check_sensors), and serialise a
// config snapshot that carries the new per-type fields. Pure logic -- no Pico SDK,
// no hardware, no Python (the patch is built in C with the same CBOR writer).
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "savia/cbor.h"
#include "savia/ble_codec.h"
#include "savia/config.h"
#include "savia/pinmap.h"
#include "savia/types.h"

// Build {v:1, op:"set", sensors:[...]} covering all four sensor molds.
static size_t build_patch(uint8_t *buf, size_t cap) {
    cbor_writer_t w;
    cbor_w_init(&w, buf, cap);
    cbor_w_map(&w, 3);
    cbor_w_textz(&w, "v");  cbor_w_uint(&w, 1);
    cbor_w_textz(&w, "op"); cbor_w_textz(&w, "set");
    cbor_w_textz(&w, "sensors");
    cbor_w_array(&w, 4);

    // 0: AquaCheck SDI-12 on GP2, addr '0' (fixed layout) + a per-sensor 10-min cadence.
    cbor_w_map(&w, 4);
    cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 2);
    cbor_w_textz(&w, "type"); cbor_w_textz(&w, "sdi12_aquacheck");
    cbor_w_textz(&w, "addr"); cbor_w_textz(&w, "0");
    cbor_w_textz(&w, "interval_s"); cbor_w_uint(&w, 600);

    // 1: analog linear on GP26, soil_temperature, value = 0.1*raw - 40.
    cbor_w_map(&w, 5);
    cbor_w_textz(&w, "gpio");   cbor_w_uint(&w, 26);
    cbor_w_textz(&w, "type");   cbor_w_textz(&w, "analog_linear");
    cbor_w_textz(&w, "kind");   cbor_w_textz(&w, "soil_temperature");
    cbor_w_textz(&w, "scale");  cbor_w_double(&w, 0.1);
    cbor_w_textz(&w, "offset"); cbor_w_double(&w, -40.0);

    // 2: generic SDI-12 on GP6, addr '1', two installer-labelled channels.
    cbor_w_map(&w, 4);
    cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 6);
    cbor_w_textz(&w, "type"); cbor_w_textz(&w, "sdi12_generic");
    cbor_w_textz(&w, "addr"); cbor_w_textz(&w, "1");
    cbor_w_textz(&w, "chan");
    cbor_w_array(&w, 2);
    cbor_w_map(&w, 2); cbor_w_textz(&w, "kind"); cbor_w_textz(&w, "soil_moisture"); cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, 10);
    cbor_w_map(&w, 2); cbor_w_textz(&w, "kind"); cbor_w_textz(&w, "soil_moisture"); cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, 30);

    // 3: DS18B20 1-Wire on GP7, kind omitted -> defaults to soil_temperature.
    cbor_w_map(&w, 3);
    cbor_w_textz(&w, "gpio");     cbor_w_uint(&w, 7);
    cbor_w_textz(&w, "type");     cbor_w_textz(&w, "onewire_ds18b20");
    cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, 5);

    assert(!w.overflow);
    return w.len;
}

int main(void) {
    uint8_t buf[512];
    size_t n = build_patch(buf, sizeof(buf));

    // --- parse the sensors[] patch ---
    ble_config_patch_t cp;
    assert(ble_parse_config_patch(buf, n, &cp));
    assert(cp.ok && cp.version == 1 && strcmp(cp.op, "set") == 0);
    assert(cp.has_sensors && cp.sensor_count == 4);

    assert(cp.sensors[0].type == SENSOR_SDI12_AQUACHECK &&
           cp.sensors[0].gpio == 2 && cp.sensors[0].address == '0');
    assert(cp.sensors[0].sample_interval_s == 600);   // per-sensor cadence parsed
    assert(cp.sensors[1].sample_interval_s == 0);      // omitted -> 0 (follow global capture_s)

    assert(cp.sensors[1].type == SENSOR_ANALOG_LINEAR && cp.sensors[1].gpio == 26);
    assert(cp.sensors[1].kind == READING_SOIL_TEMPERATURE);
    assert(cp.sensors[1].map.analog.scale  > 0.0999f && cp.sensors[1].map.analog.scale  < 0.1001f);
    assert(cp.sensors[1].map.analog.offset > -40.001f && cp.sensors[1].map.analog.offset < -39.999f);

    assert(cp.sensors[2].type == SENSOR_SDI12_GENERIC &&
           cp.sensors[2].gpio == 6 && cp.sensors[2].address == '1');
    assert(cp.sensors[2].map.sdi12.count == 2);
    assert(cp.sensors[2].map.sdi12.ch[0].kind == READING_SOIL_MOISTURE &&
           cp.sensors[2].map.sdi12.ch[0].depth_cm == 10);
    assert(cp.sensors[2].map.sdi12.ch[1].depth_cm == 30);

    assert(cp.sensors[3].type == SENSOR_ONEWIRE_DS18B20 && cp.sensors[3].gpio == 7);
    assert(cp.sensors[3].kind == READING_SOIL_TEMPERATURE && cp.sensors[3].depth_cm == 5);  // default kind
    printf("test_sensors: sensors[] patch parse OK (4 types, channels + analog map)\n");

    // --- atomic validation against a default config's reservations ---
    station_config_t cfg;
    config_load_defaults(&cfg);
    int bad = 99;
    assert(pinmap_check_sensors(&cfg, cp.sensors, cp.sensor_count, &bad) == SAVIA_PIN_ASSIGN_OK && bad == -1);

    savia_sensor_slot_t analog_on_bad_pin = cp.sensors[1];
    analog_on_bad_pin.gpio = 5;     // GP5: no ADC -> INCAPABLE at slot 0
    assert(pinmap_check_sensors(&cfg, &analog_on_bad_pin, 1, &bad) == SAVIA_PIN_ASSIGN_INCAPABLE && bad == 0);
    printf("test_sensors: atomic pin validation OK (ok set + ADC-only analog)\n");

    // --- serialize a snapshot carrying the new fields (smoke: valid CBOR) ---
    for (uint8_t i = 0; i < cp.sensor_count; i++) cfg.sensors[i] = cp.sensors[i];
    cfg.sensor_count = cp.sensor_count;
    savia_device_id_t dev = { .model = "Raspberry Pi Pico WH", .mcu = "RP2040", .fw = "0.1.0-c" };
    uint8_t snap[1024];
    size_t sl = ble_serialize_config(&dev, &cfg, snap, sizeof(snap));
    assert(sl > 0);
    cbor_reader_t r; cbor_r_init(&r, snap, sl);
    uint64_t mc; assert(cbor_r_map(&r, &mc));   // top-level is a well-formed CBOR map
    printf("test_sensors: config snapshot serialize OK (%zu B)\n", sl);

    // --- regression: a slot carrying BOTH union arms must not corrupt the union ---
    // (a generic SDI-12 slot that also carries analog scale/offset, stale or hostile).
    {
        uint8_t b[256];
        cbor_writer_t w; cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "v");  cbor_w_uint(&w, 1);
        cbor_w_textz(&w, "op"); cbor_w_textz(&w, "set");
        cbor_w_textz(&w, "sensors"); cbor_w_array(&w, 1);
        cbor_w_map(&w, 5);
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 6);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, "sdi12_generic");
        cbor_w_textz(&w, "chan"); cbor_w_array(&w, 1);
        cbor_w_map(&w, 2); cbor_w_textz(&w, "kind"); cbor_w_textz(&w, "soil_moisture"); cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, 10);
        cbor_w_textz(&w, "scale");  cbor_w_double(&w, 0.1);    // wrong arm -- must be ignored
        cbor_w_textz(&w, "offset"); cbor_w_double(&w, -40.0);
        assert(!w.overflow);
        ble_config_patch_t cp2;
        assert(ble_parse_config_patch(b, w.len, &cp2));
        assert(cp2.has_sensors && cp2.sensor_count == 1);
        assert(cp2.sensors[0].type == SENSOR_SDI12_GENERIC);
        assert(cp2.sensors[0].map.sdi12.count == 1);                       // NOT aliased by scale
        assert(cp2.sensors[0].map.sdi12.count <= SAVIA_SDI12_MAX_CHANNELS);
        assert(cp2.sensors[0].map.sdi12.ch[0].depth_cm == 10);
        printf("test_sensors: union aliasing guard OK (only the type's arm is committed)\n");
    }

    // --- regression: worst-case snapshot (6x generic SDI-12, 4 channels each) must
    // fit the H_CONFIG staging buffer (tmp[2048] in ble_gatt.c) ---
    {
        station_config_t big;
        config_load_defaults(&big);
        for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) {
            big.sensors[i].type = SENSOR_SDI12_GENERIC;
            big.sensors[i].gpio = (uint8_t)(6 + i);
            big.sensors[i].address = '0';
            big.sensors[i].map.sdi12.count = SAVIA_SDI12_MAX_CHANNELS;
            for (uint8_t c = 0; c < SAVIA_SDI12_MAX_CHANNELS; c++) {
                big.sensors[i].map.sdi12.ch[c].kind = READING_SOIL_MOISTURE;
                big.sensors[i].map.sdi12.ch[c].depth_cm = 60;
            }
        }
        big.sensor_count = SAVIA_MAX_SENSORS;
        uint8_t out[2048];
        size_t bl = ble_serialize_config(&dev, &big, out, sizeof(out));
        assert(bl > 0 && bl <= 2048);   // must fit ble_gatt.c H_CONFIG tmp[2048]
        printf("test_sensors: worst-case 6x generic-4ch snapshot fits 2048 B (%zu B)\n", bl);
    }

    printf("test_sensors: OK\n");
    return 0;
}
