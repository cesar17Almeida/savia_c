// Savia BLE wire codec (pure C, SDK-free, host-testable). Mirrors savia_py's
// CBOR shapes so TerraLink/Tobías' app talk to the Pico unchanged.
#ifndef SAVIA_BLE_CODEC_H
#define SAVIA_BLE_CODEC_H

#include "savia/types.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define BLE_DATA_CHUNK_BYTES 200   // matches protocol.DATA_CHUNK_BYTES

// data_response payloads (CBOR), mirroring savia_py. Return bytes written, 0 on overflow.
size_t ble_serialize_readings(const savia_reading_t *rows, size_t n,
                              uint8_t *out, size_t cap);   // [{ts_ms,port,kind,value,depth_cm}]
size_t ble_serialize_aggregations(const savia_aggregate_t *rows, size_t n,
                                  uint8_t *out, size_t cap); // [{hour_ms,port,kind,count,mean,min,max,depth_cm}]
size_t ble_serialize_predictions(const savia_prediction_t *rows, size_t n,
                                 uint8_t *out, size_t cap);  // [{ts_ms,model,kind,port,value,confidence}]
size_t ble_serialize_status(uint32_t uptime_s, uint64_t last_sync_ms,
                            uint64_t weather_updated_ms, uint8_t *out, size_t cap);
size_t ble_serialize_count(uint64_t count, uint8_t *out, size_t cap);  // {count:N}

// Parsers for the write characteristics.
bool ble_parse_time_sync(const uint8_t *buf, size_t len, uint64_t *ms_out);  // {v,op:"set",ms}
bool ble_parse_weather(const uint8_t *buf, size_t len);                      // {v,op:"upd",data:{...}}

// Frame a payload into data_response chunks {v,op:"chunk",s,t,eof,p}, calling
// emit() once per frame. Mirrors savia_py chunked_encode (empty -> 1 eof frame).
typedef void (*ble_frame_emit_fn)(const uint8_t *frame, size_t len, void *ctx);
void ble_chunk_encode(const uint8_t *payload, size_t len, size_t chunk_size,
                      ble_frame_emit_fn emit, void *ctx);

// Stateful chunking for BTstack flow control: send one frame per CAN_SEND_NOW.
size_t ble_chunk_total(size_t payload_len, size_t chunk_size);          // # of frames
size_t ble_chunk_make_frame(const uint8_t *payload, size_t len, size_t chunk_size,
                            size_t seq, uint8_t *out, size_t cap);       // build frame `seq`

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
