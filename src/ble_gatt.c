#include "savia/ble.h"
#include "savia/protocol.h"
// #include "pico/cyw43_arch.h"
// #include "btstack.h"

// BLE GATT peripheral via BTstack on the CYW43439. Serves the savia_py contract
// (protocol.h): status / time_sync / weather / data_request / data_response /
// blob_control. Same code on Pico WH and Pico 2 W (identical wireless chip).

void ble_init(const station_config_t *cfg) {
    (void)cfg;
    // TODO(hw):
    //   cyw43_arch_init();
    //   l2cap_init(); sm_init();
    //   att_server_init(profile_data, att_read_cb, att_write_cb);
    //   register the Savia service (SAVIA_SVC_UUID) + characteristics;
    //   gap_advertisements_setup(); gap_advertisements_enable(1);
    // data_request writes route to the get/count/infer handlers; replies go out
    // chunked on data_response (CBOR "chunk" frames, op = SAVIA_OP_CHUNK).
}

void ble_poll(uint32_t budget_ms) {
    (void)budget_ms;
    // TODO(hw): pump the BTstack run loop for the budget (cyw43_arch_poll in
    // poll mode, or just yield in threadsafe-background mode), handling the
    // connection, writes, force-inference, and chunked responses.
}
