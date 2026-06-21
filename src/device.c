#include "savia/device.h"
#include "savia/protocol.h"

// Board identity resolved at compile time. PICO_RP2040 / PICO_RP2350 are defined
// by the SDK; the host fallback keeps the unit tests SDK-free.
#if defined(PICO_RP2350)
static const savia_device_id_t s_id = {
    .model = "Raspberry Pi Pico 2 W", .mcu = "RP2350", .img = "pico_2w",
    .fw = SAVIA_FW_VERSION };
#elif defined(PICO_RP2040)
static const savia_device_id_t s_id = {
    .model = "Raspberry Pi Pico WH", .mcu = "RP2040", .img = "pico_wh",
    .fw = SAVIA_FW_VERSION };
#else
static const savia_device_id_t s_id = {
    .model = "Savia (host)", .mcu = "host", .img = "generic",
    .fw = SAVIA_FW_VERSION };
#endif

const savia_device_id_t *savia_device_id(void) { return &s_id; }
