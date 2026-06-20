#include "savia/ble.h"

#if SAVIA_ENABLE_BLE

// BLE GATT peripheral via BTstack on the CYW43439. Runs in the cyw43
// threadsafe-background context, so the supervisor loop keeps running while
// BLE advertises/serves. Milestone 1: advertise "Savia" + serve the `status`
// characteristic (read). data_request/response, time_sync, weather, infer next.
#include <stdio.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "savia_ble.h"          // generated from src/savia_ble.gatt
#include "savia/protocol.h"

#define APP_AD_FLAGS 0x06       // LE General Discoverable | BR/EDR not supported

// Advertising payload: flags + complete local name "Savia" + the 128-bit Savia
// service UUID (little-endian). 3 + 7 + 18 = 28 bytes (<= 31).
static const uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x06, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'S', 'a', 'v', 'i', 'a',
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x71, 0x5a,
};
static const uint8_t adv_data_len = sizeof(adv_data);

static btstack_packet_callback_registration_t hci_event_cb;
static uint8_t  status_payload[32];
static uint16_t status_payload_len;

static void build_status(void) {
    // Placeholder; full CBOR {v, fw, uptime_s, ...} once storage/clock land.
    const char *s = "savia: ok";
    status_payload_len = (uint16_t) strlen(s);
    memcpy(status_payload, s, status_payload_len);
}

static uint16_t att_read_cb(hci_con_handle_t con, uint16_t att_handle,
                            uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void) con;
    if (att_handle == ATT_CHARACTERISTIC_5A71A000_0000_0000_0000_000000000010_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob(status_payload, status_payload_len,
                                             offset, buffer, buffer_size);
    }
    return 0;
}

static int att_write_cb(hci_con_handle_t con, uint16_t att_handle, uint16_t tx_mode,
                        uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void) con; (void) att_handle; (void) tx_mode;
    (void) offset; (void) buffer; (void) buffer_size;
    return 0;  // no writable characteristics yet
}

static void hci_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void) channel; (void) size;
    if (type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("BLE: up, advertising as 'Savia'\n");
            }
            break;
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) ==
                HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                printf("BLE: central connected\n");
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("BLE: disconnected\n");
            break;
        default:
            break;
    }
}

void ble_init(const station_config_t *cfg) {
    (void) cfg;
    if (cyw43_arch_init()) {
        printf("BLE: cyw43_arch_init FAILED\n");
        return;
    }
    build_status();

    l2cap_init();
    sm_init();
    att_server_init(profile_data, att_read_cb, att_write_cb);

    hci_event_cb.callback = &hci_handler;
    hci_add_event_handler(&hci_event_cb);

    uint16_t adv_int_min = 0x0030, adv_int_max = 0x0030;
    bd_addr_t null_addr;
    memset(null_addr, 0, sizeof(null_addr));
    gap_advertisements_set_params(adv_int_min, adv_int_max, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t *) adv_data);
    gap_advertisements_enable(1);

    hci_power_control(HCI_POWER_ON);
    printf("BLE: Savia GATT init done\n");
}

void ble_poll(uint32_t budget_ms) {
    // threadsafe-background: BTstack runs in the cyw43 async context; nothing
    // to pump from the supervisor here.
    (void) budget_ms;
}

#else  // SAVIA_ENABLE_BLE == 0

// BLE not linked (scaffold / off-device builds). Keep the interface so the rest
// of the firmware links unchanged.
void ble_init(const station_config_t *cfg) { (void) cfg; }
void ble_poll(uint32_t budget_ms) { (void) budget_ms; }

#endif
