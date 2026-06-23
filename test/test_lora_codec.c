// Host test for the LoRa byte codec (no SDK, no hardware). Mirrors savia_py's
// tests/connectivity/test_lora_codec.py -- both ends must agree on the wire.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "savia/lora_codec.h"

static void test_downlink_roundtrip_full_window(void) {
    float past[48], future[24];
    for (int i = 0; i < 48; i++) past[i] = 20.0f + (i % 5);
    for (int i = 0; i < 24; i++) future[i] = 22.0f - (i % 3);

    uint8_t buf[LORA_DOWNLINK_MAX];
    size_t n = lora_encode_downlink(past, 48, future, 24, true, 1781012037344ULL,
                                    buf, sizeof buf);
    assert(n == 7 + 48 + 24);
    assert(n <= 115);   // fits in a DR3+ downlink

    lora_downlink_t w;
    assert(lora_decode_downlink(buf, n, &w));
    assert(w.n_past == 48 && w.n_future == 24);
    assert(w.has_time && w.time_ms == 1781012037000ULL);   // second resolution
    assert(w.past_ta[0] == 20.0f);
    assert(w.future_ta[5] == 20.0f);   // 22 - (5 % 3) = 20
    printf("test_lora_codec: downlink full-window roundtrip OK\n");
}

static void test_downlink_negative_temperatures(void) {
    float past[] = { -7.0f, -1.0f };
    float future[] = { -12.0f };
    uint8_t buf[LORA_DOWNLINK_MAX];
    size_t n = lora_encode_downlink(past, 2, future, 1, false, 0, buf, sizeof buf);
    lora_downlink_t w;
    assert(lora_decode_downlink(buf, n, &w));
    assert(w.past_ta[0] == -7.0f && w.past_ta[1] == -1.0f);
    assert(w.future_ta[0] == -12.0f);
    assert(!w.has_time);
    printf("test_lora_codec: negative temps + no clock OK\n");
}

static void test_decode_rejects_bad_frames(void) {
    lora_downlink_t w;
    uint8_t too_short[] = { 0x01, 0x00 };
    assert(!lora_decode_downlink(too_short, sizeof too_short, &w));
    uint8_t bad_version[13] = { 0x99 };
    assert(!lora_decode_downlink(bad_version, sizeof bad_version, &w));
    // n_past beyond the LSTM window must be rejected (fixed buffers).
    uint8_t oversized[7] = { 0x01, 0, 0, 0, 0, 200, 0 };
    assert(!lora_decode_downlink(oversized, sizeof oversized, &w));
    // truncated body: header says 2 past but only 1 byte follows.
    uint8_t truncated[8] = { 0x01, 0, 0, 0, 0, 2, 0, 10 };
    assert(!lora_decode_downlink(truncated, sizeof truncated, &w));
    printf("test_lora_codec: malformed-frame rejection OK\n");
}

static void test_uplink_roundtrip(void) {
    uint8_t buf[LORA_UPLINK_LEN];
    size_t n = lora_encode_uplink(true, 0.612f, buf, sizeof buf);
    assert(n == 3);
    bool has; float hs30;
    assert(lora_decode_uplink(buf, n, &has, &hs30));
    assert(has && hs30 > 0.611f && hs30 < 0.613f);

    n = lora_encode_uplink(false, 0.0f, buf, sizeof buf);
    assert(lora_decode_uplink(buf, n, &has, &hs30));
    assert(!has);   // 0xFFFF sentinel
    printf("test_lora_codec: uplink roundtrip + unknown OK\n");
}

int main(void) {
    test_downlink_roundtrip_full_window();
    test_downlink_negative_temperatures();
    test_decode_rejects_bad_frames();
    test_uplink_roundtrip();
    printf("ALL test_lora_codec PASSED\n");
    return 0;
}
