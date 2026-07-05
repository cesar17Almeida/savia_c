// Host test for the LoRa wire-v2 codec. The GOLDEN byte vectors below are the
// cross-implementation contract: savia-cloud/tests/test_codec.py asserts the SAME
// literal bytes. If you change one side, change both.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "savia/lora_codec.h"

static void expect_bytes(const uint8_t *got, size_t n,
                         const uint8_t *want, size_t wn, const char *what) {
    if (n != wn || memcmp(got, want, wn) != 0) {
        printf("FAIL %s: got %zu bytes:", what, n);
        for (size_t i = 0; i < n; i++) printf(" %02X", got[i]);
        printf("\n            want %zu bytes:", wn);
        for (size_t i = 0; i < wn; i++) printf(" %02X", want[i]);
        printf("\n");
        assert(0);
    }
}

int main(void) {
    uint8_t buf[128];

    // --- GOLDEN: uplink FORECAST (hs30_min = 0.736 -> 736 = 0x02E0) ---
    {
        size_t n = lora_encode_uplink_forecast(true, 0.736f, buf, sizeof buf);
        const uint8_t want[] = { 0x02, 0x01, 0x02, 0xE0 };
        expect_bytes(buf, n, want, sizeof want, "up_forecast");
        n = lora_encode_uplink_forecast(false, 0.0f, buf, sizeof buf);
        const uint8_t want2[] = { 0x02, 0x01, 0xFF, 0xFF };
        expect_bytes(buf, n, want2, sizeof want2, "up_forecast_unknown");
        bool has; float v;
        assert(lora_decode_uplink_forecast(want, sizeof want, &has, &v));
        assert(has && fabsf(v - 0.736f) < 0.0006f);
    }
    printf("test_lora_codec: v2 forecast golden OK\n");

    // --- GOLDEN: uplink SOIL, 2 records ---
    // rec1: ts=1782000000, hs10=0.821, hs30=0.740, ta=21.5C
    // rec2: ts=1782003600, hs10 missing, hs30=0.741, ta=-2.5C
    {
        lora_soil_rec_t recs[2] = {
            { .ts_hour_s = 1782000000u, .has_hs10 = true, .hs10 = 0.821f,
              .has_hs30 = true, .hs30 = 0.740f, .has_ta = true, .ta = 21.5f },
            { .ts_hour_s = 1782003600u, .has_hs10 = false,
              .has_hs30 = true, .hs30 = 0.741f, .has_ta = true, .ta = -2.5f },
        };
        size_t n = lora_encode_uplink_soil(recs, 2, buf, sizeof buf);
        const uint8_t want[] = {
            0x02, 0x02, 0x02,
            0x6A, 0x37, 0x29, 0x80,  0x03, 0x35,  0x02, 0xE4,  0x00, 0xD7,
            0x6A, 0x37, 0x37, 0x90,  0xFF, 0xFF,  0x02, 0xE5,  0xFF, 0xE7,
        };
        expect_bytes(buf, n, want, sizeof want, "up_soil");
        // bounds: 0 or >4 records rejected; 4 records = 43 B (fits SF12's 51 B)
        assert(lora_encode_uplink_soil(recs, 0, buf, sizeof buf) == 0);
        lora_soil_rec_t four[4] = { recs[0], recs[1], recs[0], recs[1] };
        assert(lora_encode_uplink_soil(four, 4, buf, sizeof buf) == 43);
        assert(lora_encode_uplink_soil(four, 5, buf, sizeof buf) == 0);
    }
    printf("test_lora_codec: v2 soil golden OK\n");

    // --- GOLDEN: uplink COORDS (39.4699750, -0.3762881, offset +120 min) ---
    {
        size_t n = lora_encode_uplink_coords(394699750, -3762881, 120, buf, sizeof buf);
        const uint8_t want[] = { 0x02, 0x03, 0x17, 0x86, 0xA3, 0xE6,
                                 0xFF, 0xC6, 0x95, 0x3F, 0x00, 0x78 };
        expect_bytes(buf, n, want, sizeof want, "up_coords");
    }
    // --- GOLDEN: uplink CFG_ACK (3 applied, 1 rejected) ---
    {
        size_t n = lora_encode_uplink_cfg_ack(3, 1, buf, sizeof buf);
        const uint8_t want[] = { 0x02, 0x04, 0x03, 0x01 };
        expect_bytes(buf, n, want, sizeof want, "up_cfg_ack");
    }
    printf("test_lora_codec: v2 coords + cfg_ack golden OK\n");

    // --- downlink TIME_TA: roundtrip + GOLDEN pure-clock (n=0) ---
    {
        float past[48], future[24];
        for (int i = 0; i < 48; i++) past[i] = (float)(i - 10);       // -10..37 C
        for (int i = 0; i < 24; i++) future[i] = (float)(20 + i % 5);
        size_t n = lora_encode_downlink_time_ta(past, 48, future, 24,
                                                true, 1782000000000ULL, buf, sizeof buf);
        assert(n == 8u + 48 + 24);
        lora_downlink_t d;
        assert(lora_decode_downlink(buf, n, &d));
        assert(d.type == LORA_DN_TIME_TA);
        assert(d.has_time && d.time_ms == 1782000000000ULL);
        assert(d.n_past == 48 && d.n_future == 24);
        assert(d.past_ta[0] == -10.0f && d.future_ta[0] == 20.0f);

        // GOLDEN pure clock sync: 8 bytes, keeps the weather cache untouched.
        n = lora_encode_downlink_time_ta(NULL, 0, NULL, 0, true, 1782000000000ULL,
                                         buf, sizeof buf);
        const uint8_t want[] = { 0x02, 0x01, 0x6A, 0x37, 0x29, 0x80, 0x00, 0x00 };
        expect_bytes(buf, n, want, sizeof want, "dn_time_pure");
        assert(lora_decode_downlink(buf, n, &d) && d.n_past == 0 && d.n_future == 0);
    }
    printf("test_lora_codec: v2 time_ta OK\n");

    // --- downlink CONFIG TLV: golden frame + apply with clamps ---
    {
        // sleep_s=600, daily_hour=21, inference_mode=local, utc_offset=+120,
        // irrigation_hour=25 (REJECTED: >23), unknown id 0x7F (skipped).
        const uint8_t frame[] = {
            0x02, 0x02,
            0x01, 0x04, 0x00, 0x00, 0x02, 0x58,
            0x04, 0x01, 0x15,
            0x06, 0x01, 0x01,
            0x07, 0x02, 0x00, 0x78,
            0x08, 0x01, 0x19,
            0x7F, 0x02, 0xAA, 0xBB,
        };
        lora_downlink_t d;
        assert(lora_decode_downlink(frame, sizeof frame, &d));
        assert(d.type == LORA_DN_CONFIG);
        assert(d.tlv_len == sizeof frame - 2);

        station_config_t cfg;
        config_load_defaults(&cfg);
        uint8_t ok = 0, bad = 0;
        assert(lora_apply_config_tlv(d.tlv, d.tlv_len, &cfg, &ok, &bad));
        assert(ok == 4 && bad == 1);
        assert(cfg.sleep_seconds == 600);
        assert(cfg.daily_hour == 21);
        assert(cfg.inference_mode == SAVIA_INFER_LOCAL);
        assert(cfg.utc_offset_min == 120);
        assert(cfg.irrigation_hour == 6);      // untouched default

        // negative offset via i16 two's complement: -300 = 0xFED4
        const uint8_t neg[] = { 0x07, 0x02, 0xFE, 0xD4 };
        assert(lora_apply_config_tlv(neg, sizeof neg, &cfg, &ok, &bad));
        assert(ok == 1 && cfg.utc_offset_min == -300);

        // coords pair (same values as the coords uplink golden)
        const uint8_t coords[] = { 0x09, 0x04, 0x17, 0x86, 0xA3, 0xE6,
                                   0x0A, 0x04, 0xFF, 0xC6, 0x95, 0x3F };
        assert(lora_apply_config_tlv(coords, sizeof coords, &cfg, &ok, &bad));
        assert(ok == 2 && cfg.has_coords);
        assert(cfg.lat_e7 == 394699750 && cfg.lon_e7 == -3762881);

        // malformed: truncated value -> wellformed=false
        const uint8_t trunc[] = { 0x01, 0x04, 0x00, 0x00 };
        assert(!lora_apply_config_tlv(trunc, sizeof trunc, &cfg, &ok, &bad));

        // bad value length for a known id -> rejected, stream continues
        const uint8_t badlen[] = { 0x04, 0x03, 0x01, 0x02, 0x03,
                                   0x0B, 0x01, 0x00 };
        assert(lora_apply_config_tlv(badlen, sizeof badlen, &cfg, &ok, &bad));
        assert(ok == 1 && bad == 1 && cfg.log_level == 0);
    }
    printf("test_lora_codec: v2 config TLV OK\n");

    // --- malformed frames rejected ---
    {
        lora_downlink_t d;
        const uint8_t v1[] = { 0x01, 0x01, 0x00 };          // old wire version
        assert(!lora_decode_downlink(v1, sizeof v1, &d));
        const uint8_t unk[] = { 0x02, 0x7E, 0x00 };         // unknown type
        assert(!lora_decode_downlink(unk, sizeof unk, &d));
        const uint8_t shrt[] = { 0x02 };                    // too short
        assert(!lora_decode_downlink(shrt, sizeof shrt, &d));
        const uint8_t overs[] = { 0x02, 0x01, 0, 0, 0, 0, 60, 0 };  // n_past 60 > 48
        assert(!lora_decode_downlink(overs, sizeof overs, &d));
    }
    printf("test_lora_codec: malformed-frame rejection OK\n");

    printf("ALL test_lora_codec PASSED\n");
    return 0;
}
