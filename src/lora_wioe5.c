#include "savia/lora.h"
// #include "hardware/uart.h"

// LoRaWAN node via a Grove Wio-E5 over UART (AT commands). The module runs the
// LoRaWAN MAC; we drive it. Same compact binary codec as savia_py's lora_codec:
// downlink = clock + TA forecast; uplink = HS30 forecast min.

bool lora_init(const station_config_t *cfg) {
    (void)cfg;
    // TODO(hw): uart_init on tx/rx pins @ 9600 8N1; AT handshake; configure
    // OTAA (DevEUI/AppEUI/AppKey) and region EU868; AT+JOIN.
    return false;
}

bool lora_cycle(void) {
    // TODO(hw): build the uplink (forecast-min summary), AT+CMSGHEX, then parse
    // the Class-A downlink hex -> decode_downlink() -> push TA into the weather
    // cache and set the clock. Returns true if a downlink was applied.
    return false;
}
