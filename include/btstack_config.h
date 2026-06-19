// Minimal BTstack configuration for a BLE peripheral (GATT server) on the
// Pico W / Pico 2 W (CYW43439). Tuned for a small, single-connection station.
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// BTstack features
#define ENABLE_BLE
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE   // L2CAP CoC for blob transfer
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// Memory (static; no malloc on the MCU)
#define HCI_ACL_PAYLOAD_SIZE        (255 + 4)
#define MAX_NR_GATT_CLIENTS          0
#define MAX_NR_HCI_CONNECTIONS       1
#define MAX_NR_L2CAP_SERVICES        3
#define MAX_NR_L2CAP_CHANNELS        3
#define MAX_NR_LE_DEVICE_DB_ENTRIES  1
#define MAX_NR_SM_LOOKUP_ENTRIES     3
#define MAX_NR_WHITELIST_ENTRIES     1
#define MAX_ATT_DB_SIZE              1024

#define NVM_NUM_DEVICE_DB_ENTRIES    1
#define NVM_NUM_LINK_KEYS            1

// HCI Controller to Host Flow Control
#define HCI_HOST_ACL_PACKET_LEN     (255 + 4)
#define HCI_HOST_ACL_PACKET_NUM      3
#define HCI_HOST_SCO_PACKET_LEN      0
#define HCI_HOST_SCO_PACKET_NUM      0

#endif // BTSTACK_CONFIG_H
