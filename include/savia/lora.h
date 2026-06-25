// LoRaWAN node via a Grove Wio-E5 module (AT commands over UART). Same approach
// as savia_py: the module runs the LoRaWAN MAC; we just drive it over serial.
#ifndef SAVIA_LORA_H
#define SAVIA_LORA_H

#include "savia/config.h"
#include <stdbool.h>
#include <stdint.h>

// Live LoRa link state, surfaced to the app over BLE (status …0010). The node can
// only measure the DOWNLINK signal (what it hears from the gateway), and only after
// a confirmed uplink gets an ACK -- that is what rssi/snr report. `joined` already
// proves a gateway relayed the OTAA handshake both ways.
typedef struct {
    bool     inited;         // module replied to AT over the UART (RX/TX + power OK)
    bool     joined;         // OTAA network joined
    bool     has_signal;     // a downlink RSSI/SNR has been measured at least once
    int16_t  rssi_dbm;       // last downlink RSSI in dBm (e.g. -106); valid if has_signal
    int16_t  snr_ddb;        // last downlink SNR in deci-dB (x10, e.g. 70 = 7.0 dB)
    uint64_t last_signal_ms; // wall-clock ms when last signal seen (0 = never / clock unset)
    char     module[24];     // the module's AT+VER reply (proof it talks back); "" if silent
    uint32_t seq;            // bumped on each completed ping; the app waits for a fresh result
} lora_status_t;

bool lora_init(const station_config_t *cfg);

// One Class-A cycle: uplink a compact forecast summary, then parse any downlink
// (clock + air-temperature forecast) into the weather cache. Returns true if a
// downlink was applied. Rate-limited to one attempt per LORA_PERIOD.
bool lora_cycle(void);

// On-demand "ping TTN" for the app: (re)configure the module on tx/rx, join if
// needed, send one confirmed uplink and capture its ACK's RSSI/SNR. Ignores the
// period gate and `lora_enabled`. now_wall_ms is the epoch ms to stamp the signal
// (0 if the clock isn't set yet). Returns true if the node is joined afterwards.
bool lora_ping(uint8_t tx_gpio, uint8_t rx_gpio, uint64_t now_wall_ms);

// Current link state for the BLE status payload.
void lora_get_status(lora_status_t *out);

// Seed the last-known signal from persisted config at boot, so the app can show
// the last reading before any ping happens this power cycle.
void lora_seed_last_signal(int16_t rssi_dbm, int16_t snr_ddb, uint64_t last_signal_ms);

// --- raw AT passthrough (a BLE "AT terminal" for the app) -------------------

#define SAVIA_AT_CMD_MAX    64
#define LORA_AT_MAX_LINES   12
#define LORA_AT_LINE_MAX    48

// The result of the last raw AT command run via lora_at(). Held in the driver and
// served over BLE so the app can render it chat-style.
typedef struct {
    uint32_t seq;                                   // bumps each executed command
    char     cmd[SAVIA_AT_CMD_MAX];                 // the command that was sent
    uint8_t  count;                                 // number of reply lines captured
    char     lines[LORA_AT_MAX_LINES][LORA_AT_LINE_MAX];
} lora_at_result_t;

// Run a raw AT command on the module (opens it on tx/rx if needed) and capture its
// reply lines (idle/terminator/hard-cap framed). Stores the result; bumps the seq.
void lora_at(uint8_t tx_gpio, uint8_t rx_gpio, const char *cmd);

// Latest AT exchange for the BLE response (cmd + reply lines + seq).
void lora_get_at_result(lora_at_result_t *out);

#endif // SAVIA_LORA_H
