// BTstack config for a BLE-only peripheral (GATT server) on the CYW43439
// (Pico WH / Pico 2 W). Based on the Pico SDK's embedded reference config,
// trimmed to BLE. No malloc -> fixed sizes; flow control + buffer limits are
// required to avoid the cyw43 shared-bus overrun.
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// --- Features ---
// (ENABLE_BLE lo define el SDK vía pico_btstack_ble)
#define ENABLE_LE_PERIPHERAL
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE   // L2CAP CoC (blob transfer)
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// --- Buffers / sizes ---
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (255 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_CHANNELS  4
#define MAX_NR_L2CAP_SERVICES  3
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_LE_DEVICE_DB_ENTRIES 4

// Limit ACL/SCO buffers to avoid cyw43 shared-bus overrun
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// HCI Controller-to-Host flow control (avoids cyw43 overrun)
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// NVM (link keys / device DB)
#define NVM_NUM_DEVICE_DB_ENTRIES 16
#define NVM_NUM_LINK_KEYS 16

// No malloc -> fixed-size ATT DB
#define MAX_ATT_DB_SIZE 512

// --- HAL ---
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define HCI_RESET_RESEND_TIMEOUT_MS 1000

#endif // BTSTACK_CONFIG_H
