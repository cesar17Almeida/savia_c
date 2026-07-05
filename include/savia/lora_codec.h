// Compact binary codec for the LoRaWAN node payloads, wire v2. The Python mirror
// lives in savia-cloud (app/adapters/ttn/codec.py) -- keep both in sync via the
// shared golden byte vectors in test_lora_codec.c / tests/test_codec.py. Pure
// logic (no SDK), so it unit-tests on the host. All integers big-endian.
//
// Common header (both directions): [0]=version 0x02, [1]=message type.
//
// Uplinks (node -> backend):
//   0x01 FORECAST (mode LOCAL)   [2..3] hs30_min u16 x1000 (0xFFFF = unknown)
//   0x02 SOIL     (mode FORWARD) [2] n (1..4), then n records of 10 B:
//                                ts_hour u32 (epoch s) | hs10 u16 x1000 |
//                                hs30 u16 x1000 | ta i16 x10
//                                (0xFFFF / 0x7FFF = missing value)
//   0x03 COORDS                  [2..5] lat i32 x1e-7 | [6..9] lon i32 x1e-7 |
//                                [10..11] utc_offset_min i16
//   0x04 CFG_ACK                 [2] fields applied u8 | [3] fields rejected u8
//
// Downlinks (backend -> node):
//   0x01 TIME_TA                 [2..5] clock u32 epoch s (0 = none) | [6] n_past |
//                                [7] n_future | n_past x TA i8 | n_future x TA i8
//                                (n_past = n_future = 0 -> pure clock sync, 8 B)
//   0x02 CONFIG                  TLV sequence until end of frame:
//                                [id u8][len u8][value big-endian]
//                                unknown id -> skipped via len (forward-compat)
#ifndef SAVIA_LORA_CODEC_H
#define SAVIA_LORA_CODEC_H

#include "savia/config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LORA_CODEC_VERSION   0x02

// Message types.
#define LORA_UP_FORECAST     0x01
#define LORA_UP_SOIL         0x02
#define LORA_UP_COORDS       0x03
#define LORA_UP_CFG_ACK      0x04
#define LORA_DN_TIME_TA      0x01
#define LORA_DN_CONFIG       0x02

// The LSTM window the TA arrays feed: 48 past + 24 future hourly values.
#define LORA_TA_PAST_MAX     48
#define LORA_TA_FUTURE_MAX   24
// Largest frames: downlink = TIME_TA with both full arrays; uplink = SOIL with 4
// records (2+1+4*10 = 43 B <= 51 B, the EU868 SF12 application limit).
#define LORA_DOWNLINK_MAX    (8 + LORA_TA_PAST_MAX + LORA_TA_FUTURE_MAX)
#define LORA_SOIL_RECS_MAX   4
#define LORA_UPLINK_MAX      (3 + LORA_SOIL_RECS_MAX * 10)

// Config-patch TLV field ids (downlink 0x02). Values use the same clamps as the
// BLE patch; out-of-range fields are rejected individually (LoRa is lossy -- a
// partial apply plus a CFG_ACK beats dropping the whole frame).
#define LORA_CFG_SLEEP_S         0x01   // u32
#define LORA_CFG_DEEP_SLEEP      0x02   // u8 0/1
#define LORA_CFG_CAPTURE_S       0x03   // u32
#define LORA_CFG_DAILY_HOUR      0x04   // u8 0..23 (LOCAL)
#define LORA_CFG_LORA_PERIOD_S   0x05   // u32
#define LORA_CFG_INFERENCE_MODE  0x06   // u8 savia_inference_mode_t
#define LORA_CFG_UTC_OFFSET_MIN  0x07   // i16
#define LORA_CFG_IRRIGATION_HOUR 0x08   // u8 0..23 (LOCAL)
#define LORA_CFG_LAT             0x09   // i32 x1e-7
#define LORA_CFG_LON             0x0A   // i32 x1e-7
#define LORA_CFG_LOG_LEVEL       0x0B   // u8 0/1

// One hourly soil record for the FORWARD uplink. has_* false -> sentinel on wire.
typedef struct {
    uint32_t ts_hour_s;            // hour start, epoch seconds
    bool has_hs10, has_hs30, has_ta;
    float hs10, hs30;              // VWC 0..1
    float ta;                      // degC
} lora_soil_rec_t;

// Decoded downlink, discriminated by `type`.
typedef struct {
    uint8_t  type;                        // LORA_DN_*
    // LORA_DN_TIME_TA:
    bool     has_time;                    // false when the clock field is 0
    uint64_t time_ms;                     // epoch ms (second resolution)
    uint8_t  n_past;
    float    past_ta[LORA_TA_PAST_MAX];   // degC
    uint8_t  n_future;
    float    future_ta[LORA_TA_FUTURE_MAX];
    // LORA_DN_CONFIG: the TLV region (points into the caller's frame buffer).
    const uint8_t *tlv;
    size_t         tlv_len;
} lora_downlink_t;

// --- uplink encoders (return bytes written; 0 on overflow / bad args) --------
size_t lora_encode_uplink_forecast(bool has_hs30, float hs30_min,
                                   uint8_t *out, size_t cap);
size_t lora_encode_uplink_soil(const lora_soil_rec_t *recs, uint8_t n,
                               uint8_t *out, size_t cap);
size_t lora_encode_uplink_coords(int32_t lat_e7, int32_t lon_e7,
                                 int16_t utc_offset_min, uint8_t *out, size_t cap);
size_t lora_encode_uplink_cfg_ack(uint8_t applied, uint8_t rejected,
                                  uint8_t *out, size_t cap);

// --- downlink ---------------------------------------------------------------
// Encode a TIME_TA downlink (used by tests; the backend's Python mirror encodes
// in production). Returns bytes written, 0 on overflow/oversized arrays.
size_t lora_encode_downlink_time_ta(const float *past_ta, uint8_t n_past,
                                    const float *future_ta, uint8_t n_future,
                                    bool has_time, uint64_t time_ms,
                                    uint8_t *out, size_t cap);
// Decode any downlink. Returns false on malformed/oversized/unknown-type frames.
// For CONFIG, out->tlv points INTO `data` -- keep the buffer alive while using it.
bool lora_decode_downlink(const uint8_t *data, size_t len, lora_downlink_t *out);

// Apply a CONFIG TLV region to cfg with the same clamps as the BLE patch.
// Unknown ids are skipped; out-of-range values are rejected per-field. Returns
// false only on a malformed TLV stream (truncated header/value); *applied and
// *rejected always report the field counts for the CFG_ACK uplink.
bool lora_apply_config_tlv(const uint8_t *tlv, size_t len, station_config_t *cfg,
                           uint8_t *applied, uint8_t *rejected);

// --- uplink decoders (used by tests; production decode lives in the backend) --
bool lora_decode_uplink_forecast(const uint8_t *data, size_t len,
                                 bool *has_hs30, float *hs30_min);

#endif // SAVIA_LORA_CODEC_H
