// SDI-12 helpers split in two layers:
//   - PURE parsing of probe replies (this header + src/codec/sdi12_parse.c):
//     SDK-free, host-tested in test/test_sdi12_parse.c.
//   - The wire itself (bit-banged 1200 7E1 inverted) lives in the sensor driver
//     (src/drivers/sensor_sdi12.c, firmware-only) as sdi12_transact().
// Reply formats (measured from the real AquaCheck, see
// tools/sdi12_bringup/AQUACHECK_RESPONSES.md): "aM!" -> "atttn" (ttt = seconds to
// wait, n = value count); "aD0!" -> "a+016.9562+025.1937-002.3312" (signs glued).
#ifndef SAVIA_SDI12_H
#define SAVIA_SDI12_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Parse "atttn" (reply to aM!/aM1!). Returns true and fills delay seconds +
// value count; false on a malformed/short reply.
bool sdi12_parse_measure_hdr(const char *resp, int *delay_s, int *nvals);

// Parse the +/-glued floats of a D-command reply (skipping the leading address
// char). Appends into out[got..max); returns the new total count.
int sdi12_parse_values(const char *resp, float *out, int got, int max);

// --- raw console (op "sdi12" over BLE; mirrors the LoRa AT terminal) ---------

#define SDI12_CMD_MAX        16
#define SDI12_CONSOLE_LINES  8
#define SDI12_LINE_MAX       48

typedef struct {
    uint32_t seq;                    // bumps on each executed command
    char     cmd[SDI12_CMD_MAX];
    uint8_t  count;
    char     lines[SDI12_CONSOLE_LINES][SDI12_LINE_MAX];
} sdi12_console_result_t;

// Firmware-only (sensor driver): run one raw command on `gpio` and store the
// decoded reply for the BLE console; bumps seq LAST (seq-gated result).
void sdi12_console_run(uint8_t gpio, const char *cmd);
void sdi12_get_console_result(sdi12_console_result_t *out);

#ifdef __cplusplus
}
#endif

#endif // SAVIA_SDI12_H
