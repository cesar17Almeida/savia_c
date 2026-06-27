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
#include "pico/rand.h"
#include "btstack.h"
#include "savia_ble.h"            // generated from src/savia_ble.gatt

#include "savia/ble_codec.h"
#include "savia/device.h"
#include "savia/pinmap.h"
#include "savia/lora.h"
#include "savia/auth.h"
#include "savia/storage.h"
#include "savia/clock.h"
#include "savia/protocol.h"
#include "savia/log.h"

// --- generated ATT handles --------------------------------------------------
#define H_GAP_NAME      ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE
#define H_STATUS        ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000010_01_VALUE_HANDLE
#define H_TIME_SYNC     ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000011_01_VALUE_HANDLE
#define H_WEATHER       ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000012_01_VALUE_HANDLE
#define H_DATA_REQUEST  ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000020_01_VALUE_HANDLE
#define H_DATA_RESPONSE ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000021_01_VALUE_HANDLE
#define H_DATA_RESP_CCC ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000021_01_CLIENT_CONFIGURATION_HANDLE
#define H_BLOB_CTRL     ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000030_01_VALUE_HANDLE
#define H_BLOB_CTRL_CCC ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000030_01_CLIENT_CONFIGURATION_HANDLE
#define H_CONFIG        ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000013_01_VALUE_HANDLE
#define H_CONFIG_CCC    ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000013_01_CLIENT_CONFIGURATION_HANDLE
#define H_AUTH          ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000014_01_VALUE_HANDLE
#define H_PINMAP        ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000015_01_VALUE_HANDLE

#define APP_AD_FLAGS 0x06

// ADV packet: flags + complete local name (app-editable, built at runtime). The
// 128-bit service UUID moves to the scan response so the name has room to grow.
static uint8_t g_adv_data[31];
static uint8_t g_adv_data_len;

// Scan response: the 128-bit Savia service UUID (little-endian), so a central can
// still filter by service even though it no longer rides in the ADV packet.
static const uint8_t scan_resp_data[] = {
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x71, 0x5a,
};

// Build the ADV packet from `name`, clamped to what fits after the flags field.
static void build_adv_data(const char *name) {
    size_t nlen = name ? strlen(name) : 0;
    if (nlen == 0) { name = "Savia"; nlen = 5; }
    size_t max_name = sizeof(g_adv_data) - 3 /* flags */ - 2 /* name AD header */;
    if (nlen > max_name) nlen = max_name;
    uint8_t i = 0;
    g_adv_data[i++] = 0x02;
    g_adv_data[i++] = BLUETOOTH_DATA_TYPE_FLAGS;
    g_adv_data[i++] = APP_AD_FLAGS;
    g_adv_data[i++] = (uint8_t) (nlen + 1);
    g_adv_data[i++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(g_adv_data + i, name, nlen);
    i = (uint8_t) (i + nlen);
    g_adv_data_len = i;
}

static btstack_packet_callback_registration_t hci_event_cb;
static hci_con_handle_t g_con = HCI_CON_HANDLE_INVALID;
static bool     g_notify_on;
static bool     g_advertising;          // radio up + advertising (for the status LED)
static uint64_t g_weather_updated_ms;

// auth (0014) state: a fresh nonce per connection; authed cleared on connect.
static uint8_t  g_nonce[SAVIA_AUTH_NONCE_LEN];
static bool     g_authed;

// config (0013) state
static station_config_t *g_cfg;          // held from ble_init; app writes mutate it
static bool     g_config_notify_on;
static bool     g_config_dirty;          // set on write, drained by the supervisor loop
static bool     g_lora_ping_pending;     // app asked for a LoRa ping; supervisor runs it
static bool     g_at_pending;            // app queued a raw AT command (terminal)
static char     g_at_cmd[SAVIA_AT_CMD_MAX];
static uint8_t  g_cfg_ack[64];
static size_t   g_cfg_ack_len;
static bool     g_cfg_ack_pending;       // ack waiting for a CAN_SEND_NOW slot

// data_response staging: one serialized payload, streamed as chunks.
// 12 KB holds ~150 raw readings (~67 B each); keep in sync with q_rd below.
static uint8_t  g_resp[12288];
static size_t   g_resp_len, g_resp_seq, g_resp_total;

// query scratch (BTstack context is single-threaded -> static is fine).
// 150 = 48 h x 3 series (HS10/HS30/TA), the LSTM past window.
static savia_reading_t    q_rd[150];
static savia_aggregate_t  q_agg[160];   // 48 h x up to 3 series x 2 depths of buckets
static savia_prediction_t q_pred[32];   // holds the full 24 h LSTM forecast

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

// Dev: inject one mock reading now. kind token = hs10 | hs30 | ta.
static void mock_one(const char *kind) {
    savia_reading_t r = { .ts_ms = wall_now(), .port = 1 };
    if (strcmp(kind, "hs10") == 0)      { r.kind = READING_SOIL_MOISTURE;   r.depth_cm = 10; r.value = 0.70f; }
    else if (strcmp(kind, "hs30") == 0) { r.kind = READING_SOIL_MOISTURE;   r.depth_cm = 30; r.value = 0.74f; }
    else if (strcmp(kind, "ta") == 0)   { r.kind = READING_AIR_TEMPERATURE; r.depth_cm = 0;  r.value = 22.0f; }
    else return;
    storage_append_reading(&r);
    LOG_INFO("BLE: mock %s\n", kind);
}

// Dev: inject a synthetic 24 h HS30 forecast (kind=pred) so the dashboard can be
// tested before the off-device LSTM publishes a real one. Replaces any prior mock
// forecast so repeated calls stay idempotent.
static void mock_predictions(void) {
    storage_clear_predictions();
    uint64_t now = wall_now();
    uint64_t hour = now - (now % 3600000ULL);
    for (int i = 0; i < 24; i++) {
        savia_prediction_t p;
        memset(&p, 0, sizeof(p));
        p.ts_ms = hour + (uint64_t)(i + 1) * 3600000ULL;        // next 24 hours
        strncpy(p.model, "lstm-hs30", sizeof(p.model) - 1);
        strncpy(p.kind, "hs30_forecast", sizeof(p.kind) - 1);
        p.value = 0.412f - 0.025f * (float) i / 23.0f;          // gentle overnight decline
        storage_append_prediction(&p);
    }
    LOG_INFO("BLE: mock pred (24 h hs30_forecast)\n");
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

    if (strcmp(dr.op, SAVIA_OP_COUNT) == 0) {
        uint64_t c = 0;
        if (strcmp(dr.kind, SAVIA_KIND_RAW) == 0)       c = storage_count_raw(from, to);
        else if (strcmp(dr.kind, SAVIA_KIND_PRED) == 0) c = storage_count_pred(from, to);
        else if (strcmp(dr.kind, SAVIA_KIND_AGG) == 0)  c = storage_aggregate_hourly(from, to, 0, q_agg, sizeof(q_agg) / sizeof(q_agg[0]));
        else if (strcmp(dr.kind, SAVIA_KIND_LOGS) == 0) c = savia_log_count();
        g_resp_len = ble_serialize_count(c, g_resp, sizeof(g_resp));
    } else if (strcmp(dr.op, SAVIA_OP_CLEAR) == 0) {   // dev: wipe stored data
        storage_clear();
        LOG_INFO("BLE: cleared all data\n");
        g_resp_len = ble_serialize_count(0, g_resp, sizeof(g_resp));
    } else if (strcmp(dr.op, SAVIA_OP_MOCK) == 0) {    // dev: inject mock data
        if (strcmp(dr.kind, SAVIA_KIND_PRED) == 0) {   // synthetic 24 h LSTM forecast
            mock_predictions();
            g_resp_len = ble_serialize_count(storage_count_pred(0, UINT64_MAX), g_resp, sizeof(g_resp));
        } else {                                 // one reading (hs10|hs30|ta)
            mock_one(dr.kind);
            g_resp_len = ble_serialize_count(storage_count_raw(0, UINT64_MAX), g_resp, sizeof(g_resp));
        }
    } else if (strcmp(dr.op, SAVIA_OP_INGEST) == 0) {   // app feeds timestamped points (upsert)
        bool ok;
        size_t n = ble_parse_ingest(buf, len, q_rd, sizeof(q_rd) / sizeof(q_rd[0]), &ok);
        size_t created = 0, updated = 0;
        if (ok) {
            for (size_t i = 0; i < n; i++) {
                bool cr;
                storage_upsert_reading(&q_rd[i], &cr);
                if (cr) created++; else updated++;
            }
        }
        LOG_INFO("BLE: ingest %s (created %u, updated %u)\n",
                 ok ? "ok" : "BAD", (unsigned) created, (unsigned) updated);
        g_resp_len = ble_serialize_ingest_ok(created, updated, g_resp, sizeof(g_resp));
    } else if (strcmp(dr.op, SAVIA_OP_LORA) == 0) {   // on-demand LoRa ping (join+uplink)
        g_lora_ping_pending = true;                   // supervisor runs the blocking AT
        LOG_INFO("BLE: lora ping requested\n");
        g_resp_len = ble_serialize_count(1, g_resp, sizeof(g_resp));   // queued ack
    } else if (strcmp(dr.op, SAVIA_OP_AT) == 0) {     // raw AT terminal: queue + return last result
        if (dr.has_cmd && dr.cmd[0]) {
            strncpy(g_at_cmd, dr.cmd, sizeof g_at_cmd - 1);
            g_at_cmd[sizeof g_at_cmd - 1] = '\0';
            g_at_pending = true;
            LOG_INFO("BLE: AT queued: %s\n", g_at_cmd);
        }
        static lora_at_result_t atr;                  // ~650 B -> keep off the stack
        lora_get_at_result(&atr);
        g_resp_len = ble_serialize_at_result(&atr, g_resp, sizeof(g_resp));
    } else {  // GET
        if (strcmp(dr.kind, SAVIA_KIND_RAW) == 0) {
            size_t n = storage_query_raw(from, to, lim, q_rd, sizeof(q_rd) / sizeof(q_rd[0]));
            g_resp_len = ble_serialize_readings(q_rd, n, g_resp, sizeof(g_resp));
        } else if (strcmp(dr.kind, SAVIA_KIND_AGG) == 0) {
            size_t n = storage_aggregate_hourly(from, to, lim, q_agg, sizeof(q_agg) / sizeof(q_agg[0]));
            g_resp_len = ble_serialize_aggregations(q_agg, n, g_resp, sizeof(g_resp));
        } else if (strcmp(dr.kind, SAVIA_KIND_PRED) == 0) {
            size_t n = storage_query_pred(from, to, lim, q_pred, sizeof(q_pred) / sizeof(q_pred[0]));
            g_resp_len = ble_serialize_predictions(q_pred, n, g_resp, sizeof(g_resp));
        } else if (strcmp(dr.kind, SAVIA_KIND_LOGS) == 0) {
            unsigned ln = savia_log_count();
            const char *lines[64];
            if (ln > 64) ln = 64;
            for (unsigned i = 0; i < ln; i++) lines[i] = savia_log_line(i);
            g_resp_len = ble_serialize_logs(lines, ln, g_resp, sizeof(g_resp));
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

// --- config (0013) write ----------------------------------------------------

static void handle_config_write(const uint8_t *buf, uint16_t len) {
    ble_config_patch_t cp;
    bool ok = ble_parse_config_patch(buf, len, &cp);
    const char *err = NULL;
    if (!ok || cp.version != SAVIA_PROTOCOL_VERSION || strcmp(cp.op, SAVIA_OP_SET) != 0) {
        ok = false; err = "bad patch";
    } else if (!g_cfg) {
        ok = false; err = "no config";
    } else {
        // Stage every present field into a scratch copy and validate; commit only if
        // the WHOLE patch is valid, so a later rejection (e.g. a bad sensors[]) never
        // leaves an earlier field half-applied or persisted (cross-field atomicity).
        station_config_t next = *g_cfg;

        if (ok && cp.has_sleep_s) {
            if (cp.sleep_s < SAVIA_SLEEP_MIN_S || cp.sleep_s > SAVIA_SLEEP_MAX_S) {
                ok = false; err = "sleep_s out of range";
            } else next.sleep_seconds = cp.sleep_s;
        }
        if (ok && cp.has_deep_sleep) next.deep_sleep_enabled = cp.deep_sleep;
        if (ok && cp.has_capture_s) {
            if (cp.capture_s < SAVIA_CAPTURE_MIN_S || cp.capture_s > SAVIA_SLEEP_MAX_S) {
                ok = false; err = "capture_s out of range";
            } else next.capture_interval_s = cp.capture_s;
        }
        if (ok && cp.has_daily_hour) {
            if (cp.daily_hour > 23) {
                ok = false; err = "daily_hour out of range";
            } else next.daily_hour = cp.daily_hour;
        }
        if (ok && cp.has_name) {
            if (cp.name[0] == 0) {
                ok = false; err = "name empty";
            } else {
                strncpy(next.ble_name, cp.name, SAVIA_BLE_NAME_MAX - 1);
                next.ble_name[SAVIA_BLE_NAME_MAX - 1] = 0;
            }
        }
        if (ok && cp.has_mock) next.mock_enabled = cp.mock;
        if (ok && cp.has_log_level) {
            if (cp.log_level > SAVIA_LOG_WARN) {
                ok = false; err = "log_level out of range";
            } else next.log_level = cp.log_level;
        }
        if (ok && cp.has_sensors) {
            // A per-sensor cadence (0 = follow capture_s) must sit in the same range
            // as the global capture interval; reject the whole table otherwise.
            for (uint8_t i = 0; ok && i < cp.sensor_count; i++) {
                uint32_t iv = cp.sensors[i].sample_interval_s;
                if (iv != 0 && (iv < SAVIA_CAPTURE_MIN_S || iv > SAVIA_SLEEP_MAX_S)) {
                    static char ierr[40];
                    snprintf(ierr, sizeof ierr, "sensor %d: interval_s out of range", i);
                    ok = false; err = ierr;
                }
            }
        }
        if (ok && cp.has_sensors) {
            // Validate the whole proposed table atomically against the pin inventory
            // (caps + reservations + intra-batch collisions) before committing any of it.
            int bad = -1;
            savia_pin_assign_t r = pinmap_check_sensors(&next, cp.sensors, cp.sensor_count, &bad);
            if (r != SAVIA_PIN_ASSIGN_OK) {
                static char serr[40];
                snprintf(serr, sizeof serr, "sensor %d: %s", bad, pinmap_assign_str(r));
                ok = false; err = serr;
            } else {
                for (uint8_t i = 0; i < cp.sensor_count; i++) next.sensors[i] = cp.sensors[i];
                for (uint8_t i = cp.sensor_count; i < SAVIA_MAX_SENSORS; i++)
                    next.sensors[i].type = SENSOR_NONE;
                next.sensor_count = cp.sensor_count;
            }
        }

        // Commit once, only if everything validated AND something actually changed
        // (no needless flash erase/program for a no-op patch).
        if (ok && memcmp(g_cfg, &next, sizeof(next)) != 0) {
            bool name_changed = strcmp(g_cfg->ble_name, next.ble_name) != 0;
            bool log_changed  = g_cfg->log_level != next.log_level;
            *g_cfg = next;
            g_config_dirty = true;                 // persisted by the supervisor loop
            if (name_changed) {
                build_adv_data(g_cfg->ble_name);
                gap_advertisements_set_data(g_adv_data_len, g_adv_data);  // applies on next adv
            }
            if (log_changed) savia_log_set_level(g_cfg->log_level);
            LOG_INFO("BLE: config applied (sensors=%s, name='%s')\n",
                     cp.has_sensors ? "yes" : "no", g_cfg->ble_name);
        }
    }
    if (!ok) LOG_INFO("BLE: bad config (%s)\n", err ? err : "?");

    uint32_t cur_s  = g_cfg ? g_cfg->sleep_seconds : 0;
    bool     cur_ds = g_cfg ? g_cfg->deep_sleep_enabled : false;
    g_cfg_ack_len = ble_serialize_config_ack(ok, cur_s, cur_ds, err, g_cfg_ack, sizeof(g_cfg_ack));
    if (g_con != HCI_CON_HANDLE_INVALID && g_config_notify_on && g_cfg_ack_len) {
        g_cfg_ack_pending = true;
        att_server_request_can_send_now_event(g_con);
    }
}

// --- auth (0014) write. The app confirms by re-reading the auth state. -------

static void handle_auth_write(const uint8_t *buf, uint16_t len) {
    ble_auth_msg_t m;
    if (!ble_parse_auth(buf, len, &m) || m.version != SAVIA_PROTOCOL_VERSION) {
        LOG_INFO("BLE: bad auth msg\n");
        return;
    }
    bool prov = g_cfg && auth_key_is_set(g_cfg->auth_key);

    if (strcmp(m.op, SAVIA_OP_AUTH_SET) == 0) {            // first-time password
        if (prov)            LOG_INFO("BLE: auth setpw rejected (already set)\n");
        else if (!m.has_key) LOG_INFO("BLE: auth setpw missing key\n");
        else if (g_cfg) {
            memcpy(g_cfg->auth_key, m.key, SAVIA_AUTH_KEY_LEN);
            g_config_dirty = true; g_authed = true;
            LOG_INFO("BLE: auth password set\n");
        }
    } else if (strcmp(m.op, SAVIA_OP_AUTH) == 0) {         // prove knowledge
        if (prov && m.has_mac) {
            uint8_t expect[SAVIA_AUTH_PROOF_LEN];
            auth_compute_proof(g_cfg->auth_key, g_nonce, expect);
            g_authed = auth_proof_equal(expect, m.mac);    // reflect THIS attempt
            LOG_INFO("BLE: auth %s\n", g_authed ? "OK" : "FAILED (bad password)");
        }
    } else if (strcmp(m.op, SAVIA_OP_AUTH_CHG) == 0) {     // change password (needs old proof)
        if (prov && m.has_old_mac && m.has_key && g_cfg) {
            uint8_t expect[SAVIA_AUTH_PROOF_LEN];
            auth_compute_proof(g_cfg->auth_key, g_nonce, expect);
            if (auth_proof_equal(expect, m.old_mac)) {
                memcpy(g_cfg->auth_key, m.key, SAVIA_AUTH_KEY_LEN);
                g_config_dirty = true; g_authed = true;
                LOG_INFO("BLE: auth password changed\n");
            } else {
                g_authed = false;                          // bad old proof re-locks
                LOG_INFO("BLE: auth change FAILED (bad old password)\n");
            }
        }
    } else {
        LOG_INFO("BLE: bad auth op\n");
    }
}

// --- ATT callbacks ----------------------------------------------------------

static uint16_t att_read_cb(hci_con_handle_t con, uint16_t att_handle,
                            uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void) con;
    if (att_handle == H_GAP_NAME) {   // standard Device Name (0x2A00) == advertised name
        const char *name = (g_cfg && g_cfg->ble_name[0]) ? g_cfg->ble_name : "Savia";
        return att_read_callback_handle_blob((const uint8_t *) name, strlen(name),
                                             offset, buffer, buffer_size);
    }
    if (att_handle == H_STATUS) {
        uint8_t tmp[192];   // +lora{} block
        uint32_t up_s = (uint32_t)(to_ms_since_boot(get_absolute_time()) / 1000);
        lora_status_t ls; lora_get_status(&ls);
        size_t n = ble_serialize_status(up_s, clock_last_sync_ms(),
                                        g_weather_updated_ms, &ls, tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, n, offset, buffer, buffer_size);
    }
    if (att_handle == H_CONFIG) {
        // Worst case ~1.2 KB: 6 generic SDI-12 slots x 4 channels each carry their
        // chan[] maps; a 512 B buffer would overflow -> serialize returns 0 -> empty read.
        static uint8_t tmp[2048];   // snapshot: device identity + settings + sensors
        size_t n = ble_serialize_config(savia_device_id(), g_cfg, tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, n, offset, buffer, buffer_size);
    }
    if (att_handle == H_AUTH) {
        static uint8_t tmp[96];
        bool prov = g_cfg && auth_key_is_set(g_cfg->auth_key);
        size_t n = ble_serialize_auth_state(prov, g_authed, g_nonce, SAVIA_AUTH_NONCE_LEN,
                                            tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, n, offset, buffer, buffer_size);
    }
    if (att_handle == H_PINMAP) {
        static uint8_t tmp[2048];   // GPIO inventory: 30 pins, ~1.1 KB measured
        size_t n = ble_serialize_pinmap(g_cfg, tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, n, offset, buffer, buffer_size);
    }
    return 0;
}

static int att_write_cb(hci_con_handle_t con, uint16_t att_handle, uint16_t tx_mode,
                        uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void) con; (void) tx_mode; (void) offset;
    LOG_DEBUG("BLE: write handle=0x%04x len=%u\n", att_handle, buffer_size);
    log_hexdump("  <- phone", buffer, buffer_size);

    // Locked while provisioned & not authenticated: only the auth char accepts writes.
    bool locked = g_cfg && auth_key_is_set(g_cfg->auth_key) && !g_authed;
    if (locked && att_handle != H_AUTH) {
        LOG_INFO("BLE: write locked (auth required)\n");
        return 0;
    }

    if (att_handle == H_AUTH) {
        handle_auth_write(buffer, buffer_size);
    } else if (att_handle == H_TIME_SYNC) {
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
    } else if (att_handle == H_CONFIG) {
        handle_config_write(buffer, buffer_size);
    } else if (att_handle == H_CONFIG_CCC) {
        g_config_notify_on = (buffer_size >= 2) && (little_endian_read_16(buffer, 0) & 1);
        LOG_INFO("BLE: config notify %s\n", g_config_notify_on ? "ON" : "OFF");
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
                LOG_INFO("BLE: up, advertising as '%s'\n", g_cfg ? g_cfg->ble_name : "Savia");
            break;
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) ==
                HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                g_con = hci_subevent_le_connection_complete_get_connection_handle(packet);
                for (int i = 0; i < SAVIA_AUTH_NONCE_LEN; i += 4) {   // fresh challenge
                    uint32_t rnd = get_rand_32();
                    memcpy(g_nonce + i, &rnd, 4);
                }
                g_authed = false;
                LOG_INFO("BLE: central connected\n");
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            g_con = HCI_CON_HANDLE_INVALID;
            g_notify_on = false;
            g_config_notify_on = false;
            g_cfg_ack_pending = false;
            g_lora_ping_pending = false;   // a queued ping dies with the client that asked
            g_at_pending = false;
            g_resp_seq = g_resp_total = 0;
            LOG_INFO("BLE: disconnected\n");
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            // config ack takes priority; a data_response stream re-requests below.
            if (g_cfg_ack_pending) {
                g_cfg_ack_pending = false;
                if (g_cfg_ack_len) att_server_notify(g_con, H_CONFIG, g_cfg_ack,
                                                     (uint16_t) g_cfg_ack_len);
                if (g_resp_seq < g_resp_total) att_server_request_can_send_now_event(g_con);
            } else {
                send_next_chunk();
            }
            break;
        default:
            break;
    }
}

static bool g_stack_inited;   // l2cap/sm/att/handlers are set up once, not per resume

// Bring the radio + GATT server + advertising up. Safe to call again after a
// radio_suspend(): the one-time stack init is guarded so handlers aren't doubled.
static void ble_bringup(void) {
    if (cyw43_arch_init()) {
        LOG_INFO("BLE: cyw43_arch_init FAILED\n");
        return;
    }
    if (!g_stack_inited) {
        l2cap_init();
        sm_init();
        att_server_init(profile_data, att_read_cb, att_write_cb);
        hci_event_cb.callback = &packet_handler;
        hci_add_event_handler(&hci_event_cb);
        att_server_register_packet_handler(packet_handler);
        g_stack_inited = true;
    }

    uint16_t adv_int_min = 0x0030, adv_int_max = 0x0030;
    bd_addr_t null_addr;
    memset(null_addr, 0, sizeof(null_addr));
    gap_advertisements_set_params(adv_int_min, adv_int_max, 0, 0, null_addr, 0x07, 0x00);
    build_adv_data(g_cfg ? g_cfg->ble_name : "Savia");
    gap_advertisements_set_data(g_adv_data_len, g_adv_data);
    gap_scan_response_set_data(sizeof(scan_resp_data), (uint8_t *) scan_resp_data);
    gap_advertisements_enable(1);
    hci_power_control(HCI_POWER_ON);
    g_advertising = true;
}

void ble_init(station_config_t *cfg) {
    g_cfg = cfg;
    ble_bringup();
    LOG_INFO("BLE: Savia GATT ready (status/time_sync/weather/config/data_request/data_response)\n");
}

void ble_radio_suspend(void) {
    gap_advertisements_enable(0);
    g_advertising = false;
    hci_power_control(HCI_POWER_OFF);
    cyw43_arch_deinit();                 // power down the wireless chip (the big draw)
    LOG_INFO("BLE: radio suspended (deep sleep)\n");
}

void ble_radio_resume(void) {
    ble_bringup();
    LOG_INFO("BLE: radio resumed, advertising as '%s'\n", g_cfg ? g_cfg->ble_name : "Savia");
}

void ble_poll(uint32_t budget_ms) {
    // threadsafe-background: BTstack runs in the cyw43 async context.
    (void) budget_ms;
}

bool ble_take_config_dirty(void) {
    if (!g_config_dirty) return false;
    g_config_dirty = false;
    return true;
}

bool ble_take_lora_ping(void) {
    if (!g_lora_ping_pending) return false;
    g_lora_ping_pending = false;
    return true;
}

bool ble_lora_ping_pending(void) { return g_lora_ping_pending; }

bool ble_take_lora_at(char *cmd, size_t cap) {
    if (!g_at_pending) return false;
    g_at_pending = false;
    if (cmd && cap) { strncpy(cmd, g_at_cmd, cap - 1); cmd[cap - 1] = '\0'; }
    return true;
}

bool ble_lora_at_pending(void) { return g_at_pending; }

bool ble_is_connected(void) { return g_con != HCI_CON_HANDLE_INVALID; }
bool ble_is_advertising(void) { return g_advertising; }

#else  // SAVIA_ENABLE_BLE == 0

void ble_init(station_config_t *cfg) { (void) cfg; }
void ble_poll(uint32_t budget_ms) { (void) budget_ms; }
bool ble_take_config_dirty(void) { return false; }
bool ble_take_lora_ping(void) { return false; }
bool ble_lora_ping_pending(void) { return false; }
bool ble_take_lora_at(char *cmd, size_t cap) { (void) cmd; (void) cap; return false; }
bool ble_lora_at_pending(void) { return false; }
void ble_radio_suspend(void) { }
void ble_radio_resume(void) { }
bool ble_is_connected(void) { return false; }
bool ble_is_advertising(void) { return false; }

#endif
