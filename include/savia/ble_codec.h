// Savia BLE wire codec (pure C, SDK-free, host-testable). Mirrors savia_py's
// CBOR shapes so TerraLink/Tobías' app talk to the Pico unchanged.
#ifndef SAVIA_BLE_CODEC_H
#define SAVIA_BLE_CODEC_H

#include "savia/types.h"
#include "savia/sdi12.h"
#include "savia/config.h"
#include "savia/device.h"
#include "savia/lora.h"
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
size_t ble_serialize_status(const station_config_t *cfg,
                            uint32_t uptime_s, uint64_t last_sync_ms,
                            uint64_t weather_updated_ms, const lora_status_t *lora,
                            uint8_t *out, size_t cap);
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
    bool     has_cmd;   char     cmd[SAVIA_AT_CMD_MAX];   // raw command (op:"at"/"sdi12")
    bool     has_gpio;  uint8_t  gpio;                    // probe data pin (op:"sdi12")
    bool     has_port;  uint8_t  port;                    // actuator slot port (op:"act")
    bool     has_on;    bool     on;                      // actuator target state (op:"act")
} ble_data_request_t;

// Serialize the last raw AT exchange (op:"at" response): {seq, cmd, lines:[...]}.
size_t ble_serialize_at_result(const lora_at_result_t *r, uint8_t *out, size_t cap);

// Serialize the last SDI-12 console exchange (op:"sdi12" response), same shape.
size_t ble_serialize_sdi12_result(const sdi12_console_result_t *r,
                                  uint8_t *out, size_t cap);

bool ble_parse_data_request(const uint8_t *buf, size_t len, ble_data_request_t *out);

// Parse {v, op:"ingest", data:[{ts_ms, kind, value, depth_cm?, port?}]} into
// readings (out[0..cap)). Returns the count parsed; *ok=false on malformed CBOR
// or version mismatch. Each point's kind string maps to the reading-kind enum.
size_t ble_parse_ingest(const uint8_t *buf, size_t len,
                        savia_reading_t *out, size_t cap, bool *ok);

// Ack for an ingest write (data_response): {v, op:"ingest_ok", created, updated}.
size_t ble_serialize_ingest_ok(size_t created, size_t updated, uint8_t *out, size_t cap);

// --- config characteristic (0013) -------------------------------------------

// Serialize the config snapshot (config READ): static device identity + settings
// + sensors[]. Liveness (uptime/last_sync) lives in the `status` characteristic.
// `infer_dev` is the build capability (inference_on_device()) so the app can gate
// the LOCAL mode toggle.
size_t ble_serialize_config(const savia_device_id_t *dev,
                            const station_config_t *cfg, bool infer_dev,
                            uint8_t *out, size_t cap);

// Parsed config patch (config WRITE: {v, op:"set", sleep_s?}). Extend with more
// fields (wake_gpio, sensors) as the app gains options.
typedef struct {
    bool     ok;
    int      version;
    char     op[12];
    bool     has_name;        char     name[SAVIA_BLE_NAME_MAX];
    bool     has_sleep_s;     uint32_t sleep_s;
    bool     has_deep_sleep;  bool     deep_sleep;
    bool     has_capture_s;   uint32_t capture_s;
    bool     has_daily_hour;  uint8_t  daily_hour;
    bool     has_mock;        bool     mock;
    bool     has_log_level;   uint8_t  log_level;
    bool     has_lora_period_s; uint32_t lora_period_s;
    bool     has_inference_mode; uint8_t inference_mode;   // savia_inference_mode_t
    bool     has_utc_offset;  int16_t  utc_offset_min;
    bool     has_irrigation_hour; uint8_t irrigation_hour;
    // Coords: number sets, null clears. Both must travel together to SET; either
    // null clears the pair (validated in the write path).
    bool     has_lat; bool lat_null; int32_t lat_e7;
    bool     has_lon; bool lon_null; int32_t lon_e7;
    // Full replacement of the sensor table (a sparse patch that omits "sensors"
    // leaves the slots untouched). Validated server-side via pinmap_check_sensors.
    bool     has_sensors;     uint8_t  sensor_count;
    savia_sensor_slot_t sensors[SAVIA_MAX_SENSORS];
} ble_config_patch_t;

bool ble_parse_config_patch(const uint8_t *buf, size_t len, ble_config_patch_t *out);

// --- pinmap characteristic (0015) -------------------------------------------

// Serialize the GPIO inventory (pinmap READ): {v, pins:[{gpio, state, reason,
// caps, port?}]}. state = free/in_use/reserved; reason names the system owner;
// caps is a savia_pin_cap_t bitmask. The app uses it to offer only assignable
// pins; config writes are validated server-side via pinmap_check_assign().
size_t ble_serialize_pinmap(const station_config_t *cfg, uint8_t *out, size_t cap);

// Serialize recent log lines as a CBOR array of text strings (served chunked via
// data_request kind="logs"). lines[] are NUL-terminated, oldest..newest.
size_t ble_serialize_logs(const char *const *lines, size_t n, uint8_t *out, size_t cap);

// --- auth characteristic (0014) ---------------------------------------------

// READ: {v, prov, authed, nonce:<bytes>}.
size_t ble_serialize_auth_state(bool prov, bool authed,
                                const uint8_t *nonce, size_t nonce_len,
                                uint8_t *out, size_t cap);

// WRITE: {v, op:"setpw"/"auth"/"chgpw", key?, mac?, old_mac?} (32-byte byte-strings).
typedef struct {
    bool ok;
    int  version;
    char op[12];
    bool has_key;     uint8_t key[SAVIA_AUTH_KEY_LEN];
    bool has_mac;     uint8_t mac[SAVIA_AUTH_PROOF_LEN];
    bool has_old_mac; uint8_t old_mac[SAVIA_AUTH_PROOF_LEN];
} ble_auth_msg_t;

bool ble_parse_auth(const uint8_t *buf, size_t len, ble_auth_msg_t *out);

// Ack for a config WRITE (config NOTIFY): ok -> {v,op:"config_ok",sleep_s,deep_sleep},
// else -> {v,op:"config_err",msg}.
size_t ble_serialize_config_ack(bool ok, uint32_t sleep_s, bool deep_sleep,
                                const char *err, uint8_t *out, size_t cap);

#endif // SAVIA_BLE_CODEC_H
