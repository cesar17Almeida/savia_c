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
static uint64_t g_weather_updated_ms;

// auth (0014) state: a fresh nonce per connection; authed cleared on connect.
static uint8_t  g_nonce[SAVIA_AUTH_NONCE_LEN];
static bool     g_authed;

// config (0013) state
static station_config_t *g_cfg;          // held from ble_init; app writes mutate it
static bool     g_config_notify_on;
static bool     g_config_dirty;          // set on write, drained by the supervisor loop
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

static const float m_ta[72] = {22.551666666666666,22.33666666666667,21.981666666666666,21.376666666666665,20.971666666666664,21.016666666666666,22.06833333333333,24.668333333333333,26.948333333333334,28.885,30.44333333333333,32.36166666666667,33.449999999999996,33.468333333333334,32.645,31.56,30.338333333333335,29.511666666666667,28.326666666666664,26.638333333333332,25.448333333333334,25.066666666666666,25.0,24.036666666666665,23.981666666666666,23.69666666666667,23.515,23.39833333333333,23.381666666666664,23.441666666666666,23.94333333333333,25.94333333333333,28.221666666666664,29.915000000000003,30.688333333333333,30.676666666666666,31.28,31.983333333333334,31.849999999999998,30.668333333333333,29.81,29.421666666666667,28.323333333333334,26.903333333333332,25.75,25.044999999999998,24.439999999999998,24.26,23.885,23.21166666666667,22.506666666666664,22.116666666666664,21.83666666666667,21.963333333333335,23.796666666666667,25.983333333333334,27.636666666666667,29.135,30.848333333333333,32.623333333333335,33.52166666666667,34.178333333333335,34.60166666666667,33.96,33.12166666666667,32.251666666666665,31.01,29.03,26.676666666666666,25.455,24.641666666666666,23.875};
static const float m_hs10[72] = {0.8407951349239031,0.8393684181324449,0.8378018434461062,0.8364969308105409,0.8353547611633739,0.834051491178327,0.8328273660040707,0.831625806220233,0.8371895071542131,0.8686744957537155,0.8661230011311899,0.8669119056386866,0.8618144335578378,0.8577562496825445,0.8544205896545068,0.850833963598912,0.8478275598340089,0.8461130042462846,0.845411394169964,0.844982446885388,0.8440849258820884,0.8429200603907118,0.8418569301848682,0.84070323302475,0.8396500489918427,0.8391327137319036,0.8380982867699713,0.837032108128393,0.8358348887071866,0.8347309681767353,0.8338253684357011,0.8329988841657051,0.8423207936838409,0.8718970338960771,0.8690926974624507,0.8688042357090254,0.8586142250530786,0.8512611464968153,0.8479235343368164,0.8454093732472319,0.8432240316128755,0.8412343132125817,0.8398747086930979,0.8386847118401096,0.8375874618339906,0.8361035613264071,0.8347128218467624,0.833558387002589,0.8325782928925286,0.8313437902613252,0.8300677838479169,0.8288213271579935,0.8276916775524118,0.826669435738661,0.8256483093762551,0.8245595411869343,0.8316239376029185,0.8624278935543129,0.8605434152537782,0.8624394121401747,0.8572380313358189,0.8522489353813021,0.8488007508880288,0.8457279822520806,0.8422676242393766,0.8394138511328505,0.8374519121825023,0.8358672870170021,0.8340378402699472,0.8321309192955727,0.8303283352573255,0.8289566613588111};
static const float m_hs30[72] = {0.8234399206349207,0.8231356157112527,0.8227571717230532,0.8222797032326444,0.8217127715951245,0.8210714437367304,0.8204896810307872,0.8199867834394904,0.8193337396501036,0.8589548036093418,0.8600406522064058,0.8579419530961645,0.8452106072034459,0.8361887467575935,0.8307826807295212,0.8278949737612665,0.8256552403011002,0.8241165074309978,0.8228072697883673,0.8219268565497667,0.8213675444233911,0.8211430997876858,0.821091507430998,0.8207962579617835,0.8200986464968153,0.8197458598726115,0.8192966841513676,0.8188639682539681,0.8185770995487794,0.8182503753858231,0.8175820349761526,0.8170117670946652,0.8182367780644468,0.8598649360988917,0.859359721264903,0.8589224411559618,0.8428339702760085,0.8339570063694267,0.8289032414511994,0.8260104387340789,0.8239336974506304,0.8223841008983567,0.8212068189412927,0.8203503414037129,0.8192931688740606,0.8187954605176552,0.8184123476417594,0.8182213249823383,0.8177343683651804,0.8173145675321257,0.8167754968625426,0.8162986910102309,0.8156004682136239,0.8151969342703512,0.8148147292993629,0.8141061042449343,0.8139908210639196,0.8542113923561421,0.8552318328270977,0.8548821504095598,0.8418387167433639,0.8328561759058909,0.8270948145128795,0.8230581620020555,0.8199962760779095,0.8175246853804556,0.8158976645435244,0.8146082578082016,0.8135850318471337,0.8133828298887122,0.8131,0.8129339058897789};

// Dev: inject one mock reading now. kind token = hs10 | hs30 | ta | 48h.
static void mock_one(const char *kind) {
    if (strcmp(kind, "48h") == 0) {
        // ponytail: native 48h mock loop instead of slow BLE ingest spam
        storage_clear();
        uint64_t now = wall_now();
        // ponytail: fallback if clock not set to avoid underflow
        if (!clock_is_set()) {
            now = 1770000000000ULL;
        }
        uint32_t current_hour = (now / 3600000ULL) % 24;
        
        for (int i = 48; i >= 0; i--) {
            uint64_t t = now - (i * 3600000ULL);
            int idx = 48 + current_hour - i;
            if (idx < 0) idx = 0;
            if (idx > 71) idx = 71;

            savia_reading_t r1 = { .ts_ms = t, .port = 1, .kind = READING_AIR_TEMPERATURE, .depth_cm = 0, .value = m_ta[idx] };
            savia_reading_t r2 = { .ts_ms = t, .port = 1, .kind = READING_SOIL_MOISTURE, .depth_cm = 10, .value = m_hs10[idx] };
            savia_reading_t r3 = { .ts_ms = t, .port = 1, .kind = READING_SOIL_MOISTURE, .depth_cm = 30, .value = m_hs30[idx] };
            storage_append_reading(&r1); storage_append_reading(&r2); storage_append_reading(&r3);
        }
        LOG_INFO("BLE: mock 48h history mapped to D-2..D0 CSV values\n");
        return;
    }

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
    } else {
        if (cp.has_sleep_s) {
            if (cp.sleep_s < SAVIA_SLEEP_MIN_S || cp.sleep_s > SAVIA_SLEEP_MAX_S) {
                ok = false; err = "sleep_s out of range";
            } else if (g_cfg) {
                g_cfg->sleep_seconds = cp.sleep_s;
                g_config_dirty = true;            // persisted by the supervisor loop
                LOG_INFO("BLE: config set sleep_s=%u\n", (unsigned) cp.sleep_s);
            }
        }
        if (ok && cp.has_deep_sleep && g_cfg) {
            g_cfg->deep_sleep_enabled = cp.deep_sleep;
            g_config_dirty = true;
            LOG_INFO("BLE: config set deep_sleep=%d\n", (int) cp.deep_sleep);
        }
        if (ok && cp.has_capture_s && g_cfg) {
            if (cp.capture_s < SAVIA_CAPTURE_MIN_S || cp.capture_s > SAVIA_SLEEP_MAX_S) {
                ok = false; err = "capture_s out of range";
            } else {
                g_cfg->capture_interval_s = cp.capture_s;
                g_config_dirty = true;
                LOG_INFO("BLE: config set capture_s=%u\n", (unsigned) cp.capture_s);
            }
        }
        if (ok && cp.has_daily_hour && g_cfg) {
            if (cp.daily_hour > 23) {
                ok = false; err = "daily_hour out of range";
            } else {
                g_cfg->daily_hour = cp.daily_hour;
                g_config_dirty = true;
                LOG_INFO("BLE: config set daily_hour=%u\n", (unsigned) cp.daily_hour);
            }
        }
        if (ok && cp.has_name && g_cfg) {
            if (cp.name[0] == 0) {
                ok = false; err = "name empty";
            } else {
                strncpy(g_cfg->ble_name, cp.name, SAVIA_BLE_NAME_MAX - 1);
                g_cfg->ble_name[SAVIA_BLE_NAME_MAX - 1] = 0;
                build_adv_data(g_cfg->ble_name);
                gap_advertisements_set_data(g_adv_data_len, g_adv_data);  // applies on next adv
                g_config_dirty = true;
                LOG_INFO("BLE: config set name='%s'\n", g_cfg->ble_name);
            }
        }
        if (ok && cp.has_mock && g_cfg) {
            g_cfg->mock_enabled = cp.mock;
            g_config_dirty = true;
            LOG_INFO("BLE: config set mock=%d\n", (int) cp.mock);
        }
        if (ok && cp.has_log_level && g_cfg) {
            if (cp.log_level > SAVIA_LOG_WARN) {
                ok = false; err = "log_level out of range";
            } else {
                g_cfg->log_level = cp.log_level;
                savia_log_set_level(cp.log_level);   // apply immediately
                g_config_dirty = true;
                LOG_INFO("BLE: config set log_level=%u\n", (unsigned) cp.log_level);
            }
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
        uint8_t tmp[96];
        uint32_t up_s = (uint32_t)(to_ms_since_boot(get_absolute_time()) / 1000);
        size_t n = ble_serialize_status(up_s, clock_last_sync_ms(),
                                        g_weather_updated_ms, tmp, sizeof(tmp));
        return att_read_callback_handle_blob(tmp, n, offset, buffer, buffer_size);
    }
    if (att_handle == H_CONFIG) {
        static uint8_t tmp[512];   // snapshot: device identity + settings + sensors
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
}

void ble_init(station_config_t *cfg) {
    g_cfg = cfg;
    ble_bringup();
    LOG_INFO("BLE: Savia GATT ready (status/time_sync/weather/config/data_request/data_response)\n");
}

void ble_radio_suspend(void) {
    gap_advertisements_enable(0);
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

#else  // SAVIA_ENABLE_BLE == 0

void ble_init(station_config_t *cfg) { (void) cfg; }
void ble_poll(uint32_t budget_ms) { (void) budget_ms; }
bool ble_take_config_dirty(void) { return false; }
void ble_radio_suspend(void) { }
void ble_radio_resume(void) { }

#endif
