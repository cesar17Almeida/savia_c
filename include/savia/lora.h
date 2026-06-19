// LoRaWAN node via a Grove Wio-E5 module (AT commands over UART). Same approach
// as savia_py: the module runs the LoRaWAN MAC; we just drive it over serial.
#ifndef SAVIA_LORA_H
#define SAVIA_LORA_H

#include "savia/config.h"
#include <stdbool.h>

bool lora_init(const station_config_t *cfg);

// One Class-A cycle: uplink a compact forecast summary, then parse any downlink
// (clock + air-temperature forecast) into the weather cache. Returns true if a
// downlink was applied.
bool lora_cycle(void);

#endif // SAVIA_LORA_H
