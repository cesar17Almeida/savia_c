// BLE GATT contract -- the SAME wire contract as savia_py, so TerraLink and
// Tobias' app talk to the Pico exactly as they did to the Pi. Reference:
// docs/integracion_ble_savia_tobias.md in the TFM repo.
#ifndef SAVIA_PROTOCOL_H
#define SAVIA_PROTOCOL_H

#define SAVIA_PROTOCOL_VERSION 1
#define SAVIA_FW_VERSION "0.1.0-c"   // savia_c firmware version (status payload)

// Service + characteristic UUIDs (128-bit). Base: 5a71a000-0000-0000-0000-0000000000XX
#define SAVIA_SVC_UUID            "5a71a000-0000-0000-0000-000000000001"
#define SAVIA_CHR_STATUS_UUID     "5a71a000-0000-0000-0000-000000000010"  // read, notify
#define SAVIA_CHR_TIME_SYNC_UUID  "5a71a000-0000-0000-0000-000000000011"  // write
#define SAVIA_CHR_WEATHER_UUID    "5a71a000-0000-0000-0000-000000000012"  // write
#define SAVIA_CHR_DATA_REQ_UUID   "5a71a000-0000-0000-0000-000000000020"  // write
#define SAVIA_CHR_DATA_RESP_UUID  "5a71a000-0000-0000-0000-000000000021"  // notify
#define SAVIA_CHR_BLOB_CTRL_UUID  "5a71a000-0000-0000-0000-000000000030"  // write, notify

// Control-message ops (CBOR field "op"). Match savia/ble/protocol.py.
#define SAVIA_OP_GET         "get"
#define SAVIA_OP_COUNT       "count"
#define SAVIA_OP_INFER       "infer"
#define SAVIA_OP_INFER_DONE  "infer_done"
#define SAVIA_OP_SET         "set"
#define SAVIA_OP_CHUNK       "chunk"
#define SAVIA_OP_BLOB_START  "start"
#define SAVIA_OP_BLOB_ABORT  "abort"
#define SAVIA_OP_BLOB_READY  "ready"
#define SAVIA_OP_BLOB_OK     "ok"
#define SAVIA_OP_BLOB_ERR    "err"

// data_request kinds (CBOR field "kind").
#define SAVIA_KIND_RAW   "raw"   // individual readings
#define SAVIA_KIND_AGG   "agg"   // hourly aggregates
#define SAVIA_KIND_PRED  "pred"  // model output (hs30_forecast)

// Max bytes for a control message on a single write/notify (above -> chunked).
#define SAVIA_MAX_CONTROL_MSG_BYTES 512

#endif // SAVIA_PROTOCOL_H
