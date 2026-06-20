// Savia BLE wire codec (pure C, SDK-free, host-testable). Mirrors savia_py's
// CBOR shapes so TerraLink/Tobías' app talk to the Pico unchanged.
#ifndef SAVIA_BLE_CODEC_H
#define SAVIA_BLE_CODEC_H

#include "savia/types.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define BLE_DATA_CHUNK_BYTES 200   // matches protocol.DATA_CHUNK_BYTES

// data_response raw payload: [{ts_ms,port,kind,value,depth_cm}, ...] (CBOR).
// Returns bytes written, or 0 on overflow.
size_t ble_serialize_readings(const savia_reading_t *rows, size_t n,
                              uint8_t *out, size_t cap);

// Frame a payload into data_response chunks {v,op:"chunk",s,t,eof,p}, calling
// emit() once per frame. Mirrors savia_py chunked_encode (empty -> 1 eof frame).
typedef void (*ble_frame_emit_fn)(const uint8_t *frame, size_t len, void *ctx);
void ble_chunk_encode(const uint8_t *payload, size_t len, size_t chunk_size,
                      ble_frame_emit_fn emit, void *ctx);

// Parsed data_request ({v, op:"get"/"count", kind:"raw"/"agg"/"pred", from?, to?, limit?}).
typedef struct {
    bool     ok;
    int      version;
    char     op[12];
    char     kind[8];
    bool     has_from;  uint64_t from_ms;
    bool     has_to;    uint64_t to_ms;
    bool     has_limit; uint64_t limit;
} ble_data_request_t;

bool ble_parse_data_request(const uint8_t *buf, size_t len, ble_data_request_t *out);

#endif // SAVIA_BLE_CODEC_H
