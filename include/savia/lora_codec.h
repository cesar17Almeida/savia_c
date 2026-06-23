// Compact binary codec for the LoRaWAN node payloads. Mirrors savia_py's
// lora_codec.py byte-for-byte -- the backend encodes, this node decodes. Pure
// logic (no SDK), so it unit-tests on the host.
//
// Downlink (backend -> node): fixed header + variable TA arrays
//   byte 0      version (0x01)
//   bytes 1..4  clock, epoch seconds, u32 big-endian (0 = no clock)
//   byte 5      n_past   (past TA hourly count, e.g. 48)
//   byte 6      n_future (future TA hourly count, e.g. 24)
//   n_past  x   TA degC as int8 (rounded)
//   n_future x  TA degC as int8 (rounded)
//
// Uplink (node -> backend), 3 bytes:
//   byte 0      version (0x01)
//   bytes 1..2  HS30 forecast min x1000, u16 big-endian (0xFFFF = unknown)
#ifndef SAVIA_LORA_CODEC_H
#define SAVIA_LORA_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LORA_CODEC_VERSION   0x01
// The LSTM window the TA arrays feed: 48 past + 24 future hourly values.
#define LORA_TA_PAST_MAX     48
#define LORA_TA_FUTURE_MAX   24
// Largest downlink we accept: header + both full arrays.
#define LORA_DOWNLINK_MAX    (7 + LORA_TA_PAST_MAX + LORA_TA_FUTURE_MAX)
#define LORA_UPLINK_LEN      3

// Decoded downlink, ready for clock_set + weather_set.
typedef struct {
    bool     has_time;                    // false when the clock field is 0
    uint64_t time_ms;                     // epoch ms (second resolution)
    uint8_t  n_past;
    float    past_ta[LORA_TA_PAST_MAX];   // degC
    uint8_t  n_future;
    float    future_ta[LORA_TA_FUTURE_MAX];
} lora_downlink_t;

// Encode/decode the downlink. encode returns bytes written (0 on overflow or if
// an array exceeds its max); decode returns false on a malformed/oversized frame.
size_t lora_encode_downlink(const float *past_ta, uint8_t n_past,
                            const float *future_ta, uint8_t n_future,
                            bool has_time, uint64_t time_ms,
                            uint8_t *out, size_t cap);
bool   lora_decode_downlink(const uint8_t *data, size_t len, lora_downlink_t *out);

// Encode/decode the uplink. has_hs30=false sends the "unknown" sentinel.
size_t lora_encode_uplink(bool has_hs30, float hs30_min, uint8_t *out, size_t cap);
bool   lora_decode_uplink(const uint8_t *data, size_t len,
                          bool *has_hs30, float *hs30_min);

#endif // SAVIA_LORA_CODEC_H
