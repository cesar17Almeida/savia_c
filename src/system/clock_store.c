// Persist the clock sync ring in flash so a reboot can measure the power-off
// duration (the outage). Uses the second-to-last flash sector; the last sector is
// config_store's. Kept out of clock.c so the host tests stay SDK-free. The write
// goes through flash_safe_execute so it coexists with the cyw43/BTstack context.
#include "savia/clock.h"
#include "savia/log.h"
#include <string.h>
#include <stddef.h>
#include "hardware/flash.h"
#include "pico/flash.h"

#define CLK_MAGIC   0x5356434Bu                                   // 'SVCK'
#define CLK_VERSION 1u
#define CLK_OFFSET  (PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE) // sector below config

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t len;                        // bytes of `blob` in use
    uint8_t  blob[CLOCK_RING_BLOB_MAX];  // clock_serialize_ring output
    uint32_t crc;                        // crc32 over [magic .. blob]
} clk_record_t;

_Static_assert(sizeof(clk_record_t) <= FLASH_PAGE_SIZE, "clk_record_t exceeds one flash page");

static uint32_t crc32(const uint8_t *p, size_t n) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & -(c & 1));
    }
    return ~c;
}

bool clock_store_load(void) {
    const clk_record_t *rec = (const clk_record_t *) (XIP_BASE + CLK_OFFSET);
    if (rec->magic != CLK_MAGIC || rec->version != CLK_VERSION) return false;
    if (rec->len > CLOCK_RING_BLOB_MAX) return false;
    if (crc32((const uint8_t *) rec, offsetof(clk_record_t, crc)) != rec->crc) return false;
    return clock_seed_ring(rec->blob, rec->len);
}

typedef struct { const uint8_t *blob; size_t len; } save_arg_t;

// Runs with interrupts disabled (via flash_safe_execute): no logging here. Param is
// the pre-serialized blob (snapshotted under the BLE lock by the caller).
static void do_save(void *param) {
    static uint8_t page[FLASH_PAGE_SIZE];
    const save_arg_t *a = (const save_arg_t *) param;
    size_t len = a->len > CLOCK_RING_BLOB_MAX ? CLOCK_RING_BLOB_MAX : a->len;
    memset(page, 0xff, sizeof page);
    clk_record_t *rec = (clk_record_t *) page;
    rec->magic = CLK_MAGIC;
    rec->version = CLK_VERSION;
    rec->len = (uint16_t) len;
    memcpy(rec->blob, a->blob, len);     // tail stays 0xff (part of the CRC, stable)
    rec->crc = crc32((const uint8_t *) rec, offsetof(clk_record_t, crc));
    flash_range_erase(CLK_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CLK_OFFSET, page, FLASH_PAGE_SIZE);
}

void clock_store_save(const uint8_t *blob, size_t len) {
    if (!blob) return;
    save_arg_t a = { blob, len };
    int rc = flash_safe_execute(do_save, &a, 2000);
    if (rc != PICO_OK) LOG_WARN("clock_store: flash save rc=%d\n", rc);
    else               LOG_INFO("clock_store: saved ring (%u B)\n", (unsigned) len);
}
