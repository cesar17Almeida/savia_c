// Static device identity for the "what device is this?" card (config READ).
// Picked at compile time from the target board. The app maps `model` to its own
// bundled image, so no asset key (and no image) travels over BLE.
#ifndef SAVIA_DEVICE_H
#define SAVIA_DEVICE_H

typedef struct {
    const char *model;   // human-readable board, e.g. "Raspberry Pi Pico WH"
    const char *mcu;     // chip, e.g. "RP2040"
    const char *fw;      // firmware version (SAVIA_FW_VERSION)
} savia_device_id_t;

const savia_device_id_t *savia_device_id(void);

#endif // SAVIA_DEVICE_H
