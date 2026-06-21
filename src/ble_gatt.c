#include "savia/ble.h"

#if SAVIA_ENABLE_BLE

// BLE GATT peripheral via BTstack on the CYW43439, running in the cyw43
// threadsafe-background context. Serves the savia_py data contract:
//   status        read    {v, fw, uptime_s, last_sync_ms, weather_updated_ms}
//   time_sync     write   {v, op:"set", ms}
//   weather       write   {v, op:"upd", data:{...}}
//   data_request  write   {v, op:"get"/"count", kind, from?, to?, limit?}
//   data_response notify   chunked {v, op:"chunk", s, t, eof, p}
// Blob/OTA (firmware/model) is intentionally out of scope for now.
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "savia_ble.h"            // generated from src/savia_ble.gatt

#include "savia/ble_codec.h"
#include "savia/storage.h"
#include "savia/clock.h"
#include "savia/protocol.h"
#include "savia/log.h"

// --- generated ATT handles --------------------------------------------------
#define H_STATUS        ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000010_01_VALUE_HANDLE
#define H_TIME_SYNC     ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000011_01_VALUE_HANDLE
#define H_WEATHER       ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000012_01_VALUE_HANDLE
#define H_DATA_REQUEST  ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000020_01_VALUE_HANDLE
#define H_DATA_RESPONSE ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000021_01_VALUE_HANDLE
#define H_DATA_RESP_CCC ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000021_01_CLIENT_CONFIGURATION_HANDLE
#define H_BLOB_CTRL     ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000030_01_VALUE_HANDLE
#define H_BLOB_CTRL_CCC ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000030_01_CLIENT_CONFIGURATION_HANDLE

#define APP_AD_FLAGS 0x06

static const uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x06, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'S', 'a', 'v', 'i', 'a',
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x71, 0x5a,
};
static const uint8_t adv_data_len = sizeof(adv_data);

static btstack_packet_callback_registration_t hci_event_cb;
static hci_con_handle_t g_con = HCI_CON_HANDLE_INVALID;
static bool     g_notify_on;
static uint64_t g_weather_updated_ms;

// data_response staging: one serialized payload, streamed as chunks.
static uint8_t  g_resp[4096];
static size_t   g_resp_len, g_resp_seq, g_resp_total;

// query scratch (BTstack context is single-threaded -> static is fine)
static savia_reading_t    q_rd[64];
static savia_aggregate_t  q_agg[32];
static savia_prediction_t q_pred[16];

static uint64_t wall_now(void) {
    uint64_t up = to_ms_since_boot(get_absolute_time());
    return clock_is_set() ? clock_now(up) : up;
}

// --- data_response streaming ------------------------------------------------

static void send_next_chunk(void) {
    if (g_con == HCI_CON_HANDLE_INVALID || g_resp_seq >= g_resp_total) return;
    uint8_t frame[BLE_DATA_CHUNK_BYTES + 64];
    size_t fl = ble_chunk_make_frame(g_resp, g_resp_len, BLE_DATA_CHUNK_BYTES,
                                     g_resp_seq, frame, sizeof(frame));
    g_resp_seq++;
    if (fl) att_server_notify(g_con, H_DATA_RESPONSE, frame, (uint16_t) fl);
    if (g_resp_seq < g_resp_total) att_server_request_can_send_now_event(g_con);
}

static void handle_data_request(const uint8_t *buf, uint16_t len) {
    ble_data_request_t dr;
    if (!ble_parse_data_request(buf, len, &dr)) {
        LOG_INFO("BLE: bad data_request\n");
        return;
    }
    uint64_t from = dr.has_from ? dr.from_ms : 0;
    uint64_t to   = dr.has_to ? dr.to_ms : UINT64_MAX;
    size_t   lim  = dr.has_limit ? (size_t) dr.limit : 0;

    if (strcmp(dr.op, "count") == 0) {
        uint64_t c = 0;
        if (strcmp(dr.kind, "raw") == 0)       c = storage_count_raw(from, to);
        else if (strcmp(dr.kind, "pred") == 0) c = storage_count_pred(from, to);
        else if (strcmp(dr.kind, "agg") == 0)  c = storage_aggregate_hourly(from, to, 0, q_agg, 32);
        g_resp_len = ble_serialize_count(c, g_resp, sizeof(g_resp));
    } else {  // GET
        if (strcmp(dr.kind, "raw") == 0) {
            size_t n = storage_query_raw(from, to, lim, q_rd, 64);
            g_resp_len = ble_serialize_readings(q_rd, n, g_resp, sizeof(g_resp));
        } else if (strcmp(dr.kind, "agg") == 0) {
            size_t n = storage_aggregate_hourly(from, to, lim, q_agg, 32);
            g_resp_len = ble_serialize_aggregations(q_agg, n, g_resp, sizeof(g_resp));
        } else if (strcmp(dr.kind, "pred") == 0) {
            size_t n = storage_query_pred(from, to, lim, q_pred, 16);
            g_resp_len = ble_serialize_predictions(q_pred, n, g_resp, sizeof(g_resp));
        } else {
            g_resp_len = 0;
        }
    }

    g_resp_seq = 0;
    g_resp_total = ble_chunk_total(g_resp_len, BLE_DATA_CHUNK_BYTES);
    LOG_INFO("BLE: data_request %s/%s -> %u B in %u frame(s)\n",
           dr.op, dr.kind, (unsigned) g_resp_len, (unsigned) g_resp_total);
    if (g_con != HCI_CON_HANDLE_INVALID) att_server_request_can_send_now_event(g_con);
}

// --- ATT callbacks ----------------------------------------------------------

static uint16_t att_read_cb(hci_con_handle_t con, uint16_t att_handle,
                            uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void) con;
    if (att_handle == H_STATUS) {
        uint8_t tmp[96];
        uint32_t up_s = (uint32_t)(to_ms_since_boot(get_absolute_time()) / 1000);
        size_t n = ble_serialize_status(up_s, clock_last_sync_ms(),
                                        g_weather_updated_ms, tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, n, offset, buffer, buffer_size);
    }
    return 0;
}

static int att_write_cb(hci_con_handle_t con, uint16_t att_handle, uint16_t tx_mode,
                        uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void) con; (void) tx_mode; (void) offset;
    LOG_DEBUG("BLE: write handle=0x%04x len=%u\n", att_handle, buffer_size);
    log_hexdump("  <- phone", buffer, buffer_size);
    if (att_handle == H_TIME_SYNC) {
        uint64_t ms;
        if (ble_parse_time_sync(buffer, buffer_size, &ms)) {
            clock_set(ms, to_ms_since_boot(get_absolute_time()));
            LOG_INFO("BLE: time_sync -> %llu ms\n", (unsigned long long) ms);
        } else LOG_INFO("BLE: bad time_sync\n");
    } else if (att_handle == H_WEATHER) {
        if (ble_parse_weather(buffer, buffer_size)) {
            g_weather_updated_ms = wall_now();
            LOG_INFO("BLE: weather cached\n");
        } else LOG_INFO("BLE: bad weather\n");
    } else if (att_handle == H_DATA_REQUEST) {
        handle_data_request(buffer, buffer_size);
    } else if (att_handle == H_DATA_RESP_CCC) {
        g_notify_on = (buffer_size >= 2) && (little_endian_read_16(buffer, 0) & 1);
        LOG_INFO("BLE: data_response notify %s\n", g_notify_on ? "ON" : "OFF");
    } else if (att_handle == H_BLOB_CTRL || att_handle == H_BLOB_CTRL_CCC) {
        // blob_control (firmware/model OTA): stub, present for discovery only.
    }
    return 0;
}

static void packet_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void) channel; (void) size;
    if (type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
                LOG_INFO("BLE: up, advertising as 'Savia'\n");
            break;
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) ==
                HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                g_con = hci_subevent_le_connection_complete_get_connection_handle(packet);
                LOG_INFO("BLE: central connected\n");
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            g_con = HCI_CON_HANDLE_INVALID;
            g_notify_on = false;
            g_resp_seq = g_resp_total = 0;
            LOG_INFO("BLE: disconnected\n");
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            send_next_chunk();
            break;
        default:
            break;
    }
}

void ble_init(const station_config_t *cfg) {
    (void) cfg;
    if (cyw43_arch_init()) {
        LOG_INFO("BLE: cyw43_arch_init FAILED\n");
        return;
    }
    l2cap_init();
    sm_init();
    att_server_init(profile_data, att_read_cb, att_write_cb);

    hci_event_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_event_cb);
    att_server_register_packet_handler(packet_handler);

    uint16_t adv_int_min = 0x0030, adv_int_max = 0x0030;
    bd_addr_t null_addr;
    memset(null_addr, 0, sizeof(null_addr));
    gap_advertisements_set_params(adv_int_min, adv_int_max, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t *) adv_data);
    gap_advertisements_enable(1);

    hci_power_control(HCI_POWER_ON);
    LOG_INFO("BLE: Savia GATT ready (status/time_sync/weather/data_request/data_response)\n");
}

void ble_poll(uint32_t budget_ms) {
    // threadsafe-background: BTstack runs in the cyw43 async context.
    (void) budget_ms;
}

#else  // SAVIA_ENABLE_BLE == 0

void ble_init(const station_config_t *cfg) { (void) cfg; }
void ble_poll(uint32_t budget_ms) { (void) budget_ms; }

#endif
