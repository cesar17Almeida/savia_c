// Host test for the sensor configuration path: build a config patch carrying a
// sensors[] table (all four molds), parse it (ble_parse_config_patch), validate it
// atomically against the pin inventory (pinmap_check_sensors), and serialise a
// config snapshot that carries the new per-type fields. Pure logic -- no Pico SDK,
// no hardware, no Python (the patch is built in C with the same CBOR writer).
#include <assert.h>
#include <math.h>
#include <stdint.h>
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

// Occupied slots of a slot-addressed table (holes are SENSOR_NONE).
static uint8_t slots_used(const savia_sensor_slot_t *slots) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) if (slots[i].type != SENSOR_NONE) n++;
    return n;
}

int main(void) {
    uint8_t buf[512];
    size_t n = build_patch(buf, sizeof(buf));

    // --- parse the sensors[] patch ---
    ble_config_patch_t cp;
    assert(ble_parse_config_patch(buf, n, &cp));
    assert(cp.ok && cp.version == 1 && strcmp(cp.op, "set") == 0);
    assert(cp.has_sensors && slots_used(cp.sensors) == 4);
    // Slot-addressed: four entries without an explicit port fill slots 0..3 in order.
    assert(cp.sensors[4].type == SENSOR_NONE && cp.sensors[5].type == SENSOR_NONE);

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
    assert(pinmap_check_sensors(&cfg, cp.sensors, &bad) == SAVIA_PIN_ASSIGN_OK && bad == -1);

    savia_sensor_slot_t one_bad[SAVIA_MAX_SENSORS] = {0};
    one_bad[0] = cp.sensors[1];
    one_bad[0].gpio = 5;            // GP5: no ADC -> INCAPABLE at slot 0
    assert(pinmap_check_sensors(&cfg, one_bad, &bad) == SAVIA_PIN_ASSIGN_INCAPABLE && bad == 0);
    printf("test_sensors: atomic pin validation OK (ok set + ADC-only analog)\n");

    // --- serialize a snapshot carrying the new fields (smoke: valid CBOR) ---
    for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) cfg.sensors[i] = cp.sensors[i];
    savia_device_id_t dev = { .model = "Raspberry Pi Pico WH", .mcu = "RP2040", .fw = "0.1.0-c" };
    uint8_t snap[1024];
    size_t sl = ble_serialize_config(&dev, &cfg, false, snap, sizeof(snap));
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
        assert(cp2.has_sensors && slots_used(cp2.sensors) == 1);
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
        uint8_t out[2048];
        size_t bl = ble_serialize_config(&dev, &big, false, out, sizeof(out));
        assert(bl > 0 && bl <= 2048);   // must fit ble_gatt.c H_CONFIG tmp[2048]
        printf("test_sensors: worst-case 6x generic-4ch snapshot fits 2048 B (%zu B)\n", bl);
    }

    // --- new simple types: DHT11 / HC-SR04 (gpio2) / actuator + unit label ---
    {
        uint8_t b[512];
        cbor_writer_t w; cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "v");  cbor_w_uint(&w, 1);
        cbor_w_textz(&w, "op"); cbor_w_textz(&w, "set");
        cbor_w_textz(&w, "sensors"); cbor_w_array(&w, 3);
        // DHT11 on GP8 (single pin, self-describing).
        cbor_w_map(&w, 2);
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 8);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, "dht11");
        // HC-SR04: trigger GP6 + echo GP7 + free unit label.
        cbor_w_map(&w, 5);
        cbor_w_textz(&w, "gpio");  cbor_w_uint(&w, 6);
        cbor_w_textz(&w, "gpio2"); cbor_w_uint(&w, 7);
        cbor_w_textz(&w, "type");  cbor_w_textz(&w, "hc_sr04");
        cbor_w_textz(&w, "kind");  cbor_w_textz(&w, "distance");
        cbor_w_textz(&w, "unit");  cbor_w_textz(&w, "mm");
        // Digital actuator (valve) on GP9.
        cbor_w_map(&w, 2);
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 9);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, "actuator");
        assert(!w.overflow);

        ble_config_patch_t cp3;
        assert(ble_parse_config_patch(b, w.len, &cp3));
        assert(cp3.has_sensors && slots_used(cp3.sensors) == 3);
        assert(cp3.sensors[0].type == SENSOR_DHT11 && cp3.sensors[0].gpio == 8);
        assert(cp3.sensors[0].gpio2 == SAVIA_GPIO_NONE);       // absent -> unused
        assert(cp3.sensors[1].type == SENSOR_HCSR04);
        assert(cp3.sensors[1].gpio == 6 && cp3.sensors[1].gpio2 == 7);
        assert(cp3.sensors[1].kind == READING_DISTANCE);
        assert(strcmp(cp3.sensors[1].unit, "mm") == 0);
        assert(cp3.sensors[2].type == SENSOR_ACTUATOR_DIGITAL && cp3.sensors[2].gpio == 9);

        // The set validates against the default reservations, and a snapshot
        // carrying gpio2/unit serializes to well-formed CBOR.
        int bad3 = 99;
        assert(pinmap_check_sensors(&cfg, cp3.sensors, &bad3) == SAVIA_PIN_ASSIGN_OK);
        station_config_t c3;
        config_load_defaults(&c3);
        for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) c3.sensors[i] = cp3.sensors[i];
        uint8_t snap3[1024];
        size_t sl3 = ble_serialize_config(&dev, &c3, false, snap3, sizeof(snap3));
        assert(sl3 > 0);
        cbor_reader_t r3; cbor_r_init(&r3, snap3, sl3);
        uint64_t mc3; assert(cbor_r_map(&r3, &mc3));
        printf("test_sensors: new types (dht11 / hc_sr04+gpio2 / actuator) + unit OK\n");
    }

    // --- slot addressing: a port names the slot, holes survive the round trip ---
    // This is what stops a delete from renumbering the sensors after it, and with
    // them the port every stored reading is keyed by.
    {
        uint8_t b[256];
        cbor_writer_t w; cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "v");  cbor_w_uint(&w, 1);
        cbor_w_textz(&w, "op"); cbor_w_textz(&w, "set");
        // Ports 1 and 3 only: slot 2 is the hole a deleted sensor left behind.
        cbor_w_textz(&w, "sensors"); cbor_w_array(&w, 2);
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "port"); cbor_w_uint(&w, 1);
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 8);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, "dht11");
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "port"); cbor_w_uint(&w, 3);
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 9);
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, "actuator");
        assert(!w.overflow);

        ble_config_patch_t cps;
        assert(ble_parse_config_patch(b, w.len, &cps) && cps.has_sensors);
        assert(cps.sensors[0].type == SENSOR_DHT11 && cps.sensors[0].gpio == 8);
        assert(cps.sensors[1].type == SENSOR_NONE);              // the hole stays a hole
        assert(cps.sensors[1].gpio2 == SAVIA_GPIO_NONE);         // and is not GP0
        assert(cps.sensors[2].type == SENSOR_ACTUATOR_DIGITAL && cps.sensors[2].gpio == 9);
        assert(slots_used(cps.sensors) == 2);

        // A snapshot of that table re-emits the SAME ports, hole included.
        station_config_t hc;
        config_load_defaults(&hc);
        for (uint8_t i = 0; i < SAVIA_MAX_SENSORS; i++) hc.sensors[i] = cps.sensors[i];
        uint8_t snap[512];
        size_t hl = ble_serialize_config(&dev, &hc, false, snap, sizeof(snap));
        assert(hl > 0);
        // The wire must carry port 3, not a compacted port 2.
        bool saw_port3 = false;
        for (size_t i = 0; i + 6 < hl; i++) {
            if (memcmp(snap + i, "\x64port", 5) == 0 && snap[i + 5] == 0x03) saw_port3 = true;
        }
        assert(saw_port3);
        printf("test_sensors: slot addressing OK (hole at port 2 survives round trip)\n");
    }

    // --- slot addressing: malformed port tables are rejected whole ---
    {
        uint8_t b[128];
        for (int variant = 0; variant < 2; variant++) {
            cbor_writer_t w; cbor_w_init(&w, b, sizeof(b));
            cbor_w_map(&w, 3);
            cbor_w_textz(&w, "v");  cbor_w_uint(&w, 1);
            cbor_w_textz(&w, "op"); cbor_w_textz(&w, "set");
            cbor_w_textz(&w, "sensors"); cbor_w_array(&w, 2);
            cbor_w_map(&w, 3);
            cbor_w_textz(&w, "port"); cbor_w_uint(&w, 2);
            cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 8);
            cbor_w_textz(&w, "type"); cbor_w_textz(&w, "dht11");
            cbor_w_map(&w, 3);
            // variant 0: the same port twice. variant 1: a port past the last slot.
            cbor_w_textz(&w, "port"); cbor_w_uint(&w, variant == 0 ? 2 : SAVIA_MAX_SENSORS + 1);
            cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 9);
            cbor_w_textz(&w, "type"); cbor_w_textz(&w, "actuator");
            assert(!w.overflow);
            ble_config_patch_t bad_cp;
            assert(!ble_parse_config_patch(b, w.len, &bad_cp));
        }
        printf("test_sensors: duplicate / out-of-range port rejected\n");
    }

    // --- a retired wire field must not desync the parser ---
    // An older TerraLink still sends irrigation_hour (dropped ago-2026). The key has
    // no branch left, so it takes the fallback (cbor_r_skip): it is ignored and the
    // fields AROUND it must still land. Getting this wrong would reject the whole
    // patch, so mixed app/firmware versions are worth a test of their own.
    {
        uint8_t b[128];
        cbor_writer_t w; cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 4);
        cbor_w_textz(&w, "v");               cbor_w_uint(&w, 1);
        cbor_w_textz(&w, "daily_hour");      cbor_w_uint(&w, 7);
        cbor_w_textz(&w, "irrigation_hour"); cbor_w_uint(&w, 6);   // retired
        cbor_w_textz(&w, "capture_s");       cbor_w_uint(&w, 900); // after it
        assert(!w.overflow);

        ble_config_patch_t old_cp;
        assert(ble_parse_config_patch(b, w.len, &old_cp));
        assert(old_cp.has_daily_hour && old_cp.daily_hour == 7);
        assert(!old_cp.has_daily_min);                             // absent key -> not applied
        assert(old_cp.has_capture_s && old_cp.capture_s == 900);   // parser stayed in sync
        printf("test_sensors: retired field skipped, neighbours still applied\n");
    }

    // Out-of-range wire values must never wrap or truncate into a valid setting
    // (found by fuzzing): an oversized integer rejects the patch, and a NaN or
    // huge coordinate/offset maps to a sentinel the write path refuses.
    {
        uint8_t b[128];
        cbor_writer_t w; cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "lat");            cbor_w_double(&w, NAN);
        cbor_w_textz(&w, "lon");            cbor_w_double(&w, 9064.0);
        cbor_w_textz(&w, "utc_offset_min"); cbor_w_uint(&w, 65596);   // 65536 + 60
        assert(!w.overflow);
        ble_config_patch_t lim;
        assert(ble_parse_config_patch(b, w.len, &lim));
        assert(lim.has_lat && lim.lat_e7 == INT32_MIN);
        assert(lim.has_lon && lim.lon_e7 == INT32_MIN);
        assert(lim.has_utc_offset && lim.utc_offset_min < SAVIA_UTC_OFFSET_MIN);

        const char *keys[] = { "lora_tx", "daily_hour", "v" };
        for (int i = 0; i < 3; i++) {
            cbor_w_init(&w, b, sizeof(b));
            cbor_w_map(&w, 1);
            cbor_w_textz(&w, keys[i]); cbor_w_uint(&w, 272);           // 256 + 16
            assert(!ble_parse_config_patch(b, w.len, &lim));
        }
        cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 1);
        cbor_w_textz(&w, "sleep_s"); cbor_w_uint(&w, 4294967296ull + 600);
        assert(!ble_parse_config_patch(b, w.len, &lim));

        cbor_w_init(&w, b, sizeof(b));
        cbor_w_map(&w, 1);
        cbor_w_textz(&w, "sensors"); cbor_w_array(&w, 1);
        cbor_w_map(&w, 2);
        cbor_w_textz(&w, "gpio"); cbor_w_uint(&w, 258);                // 256 + 2
        cbor_w_textz(&w, "type"); cbor_w_textz(&w, "sdi12_aquacheck");
        assert(!w.overflow);
        assert(!ble_parse_config_patch(b, w.len, &lim));

        // An ingest point with an oversized depth is dropped; its neighbour still lands.
        uint8_t ib[160];
        cbor_w_init(&w, ib, sizeof(ib));
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "v");  cbor_w_uint(&w, 1);
        cbor_w_textz(&w, "op"); cbor_w_textz(&w, "ingest");
        cbor_w_textz(&w, "data"); cbor_w_array(&w, 2);
        cbor_w_map(&w, 4);
        cbor_w_textz(&w, "ts_ms"); cbor_w_uint(&w, 1700000000000ull);
        cbor_w_textz(&w, "kind");  cbor_w_textz(&w, "soil_moisture");
        cbor_w_textz(&w, "value"); cbor_w_double(&w, 0.31);
        cbor_w_textz(&w, "depth_cm"); cbor_w_uint(&w, 300);
        cbor_w_map(&w, 3);
        cbor_w_textz(&w, "ts_ms"); cbor_w_uint(&w, 1700000060000ull);
        cbor_w_textz(&w, "kind");  cbor_w_textz(&w, "soil_moisture");
        cbor_w_textz(&w, "value"); cbor_w_double(&w, 0.32);
        assert(!w.overflow);
        savia_reading_t pts[4]; bool iok = false;
        assert(ble_parse_ingest(ib, w.len, pts, 4, &iok) == 1 && iok);
        assert(pts[0].ts_ms == 1700000060000ull);
        printf("test_sensors: oversized and non-finite wire values rejected\n");
    }

    // CBOR lengths near 2^64 must fail cleanly: pos + len used to wrap past the
    // bounds check, read beyond the write and even move the cursor backwards.
    {
        static const uint8_t evil_text[] = {
            0x7b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 'a' };
        static const uint8_t evil_bytes[] = {
            0x5b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 'a' };
        cbor_reader_t hr;
        const char *hs; const uint8_t *hp; size_t hn;
        cbor_r_init(&hr, evil_text, sizeof evil_text);
        assert(!cbor_r_text(&hr, &hs, &hn) && hr.err);
        cbor_r_init(&hr, evil_bytes, sizeof evil_bytes);
        assert(!cbor_r_bytes(&hr, &hp, &hn) && hr.err);
        cbor_r_init(&hr, evil_bytes, sizeof evil_bytes);
        assert(!cbor_r_skip(&hr) && hr.err);

        static const uint8_t evil_name[] = {
            0xa2, 0x64, 'n', 'a', 'm', 'e',
            0x7b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 'a', 'b', 'c' };
        ble_config_patch_t ev;
        assert(!ble_parse_config_patch(evil_name, sizeof evil_name, &ev));
        assert(!ev.has_name);

        // Unknown key whose value length jumps the cursor back onto that key: an
        // endless loop inside an indefinite map before the fix.
        static const uint8_t evil_loop[] = {
            0xbf, 0x63, 'z', 'z', 'z',
            0x5b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3 };
        assert(!ble_parse_config_patch(evil_loop, sizeof evil_loop, &ev));
        ble_auth_msg_t am;                         // the pre-auth characteristic too
        assert(!ble_parse_auth(evil_loop, sizeof evil_loop, &am));
        printf("test_sensors: hostile CBOR lengths rejected\n");
    }

    printf("test_sensors: OK\n");
    return 0;
}
