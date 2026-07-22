// Persist station_config_t in the last flash sector (survives reboot). Kept out
// of config.c so the host tests stay SDK-free. Write goes through
// flash_safe_execute so it coexists with the cyw43/BTstack background context.
#include "savia/config.h"
#include "savia/log.h"
#include <string.h>
#include <stddef.h>
#include "hardware/flash.h"
#include "pico/flash.h"

#define CFG_MAGIC   0x53564346u                                // 'SVCF'
#define CFG_VERSION 8u   // bumped: lora_enabled defaults ON (boot uplink = time source)
#define CFG_VERSION_PREV 7u   // same struct layout; migrated in-place on load
#define CFG_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE) // last 4 KB sector

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;             // sizeof(station_config_t), guards layout changes
    station_config_t cfg;
    uint32_t crc;              // crc32 over [magic .. cfg]
} cfg_record_t;

// The config lives in one 4 KB erase sector. Erase is per-sector but PROGRAM is per
// 256 B page, so the record may grow up to the whole sector -- we just program as
// many pages as it needs (CFG_PROG_LEN). It must fit the sector or it is truncated
// and the CRC then rejects every load.
_Static_assert(sizeof(cfg_record_t) <= FLASH_SECTOR_SIZE, "cfg_record_t exceeds one flash sector");

// Bytes actually programmed: the record rounded up to a whole number of 256 B pages.
#define CFG_PROG_LEN (((sizeof(cfg_record_t) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

static uint32_t crc32(const uint8_t *p, size_t n) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & -(c & 1));
    }
    return ~c;
}

bool config_store_load(station_config_t *cfg) {
    const cfg_record_t *rec = (const cfg_record_t *) (XIP_BASE + CFG_OFFSET);
    if (rec->magic != CFG_MAGIC ||
        (rec->version != CFG_VERSION && rec->version != CFG_VERSION_PREV) ||
        rec->size != sizeof(station_config_t)) return false;
    if (crc32((const uint8_t *) rec, offsetof(cfg_record_t, crc)) != rec->crc) return false;
    memcpy(cfg, &rec->cfg, sizeof(*cfg));
    // v7 -> v8: LoRa becomes on-by-default; everything else carries over. The
    // record is rewritten as v8 on the next config_store_save.
    if (rec->version == CFG_VERSION_PREV) cfg->lora_enabled = true;
    return true;
}

// Runs with interrupts disabled (via flash_safe_execute): no logging here.
static void do_save(void *param) {
    static uint8_t page[CFG_PROG_LEN];          // record rounded up to whole 256 B pages
    cfg_record_t *rec = (cfg_record_t *) page;
    memset(page, 0xff, sizeof(page));
    rec->magic = CFG_MAGIC;
    rec->version = CFG_VERSION;
    rec->size = (uint16_t) sizeof(station_config_t);
    memcpy(&rec->cfg, (const station_config_t *) param, sizeof(rec->cfg));
    rec->crc = crc32((const uint8_t *) rec, offsetof(cfg_record_t, crc));
    flash_range_erase(CFG_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CFG_OFFSET, page, CFG_PROG_LEN);
}

void config_store_save(const station_config_t *cfg) {
    int rc = flash_safe_execute(do_save, (void *) cfg, 2000);
    if (rc != PICO_OK) LOG_WARN("config_store: flash save rc=%d\n", rc);
    else               LOG_INFO("config_store: saved (sleep_s=%u)\n", (unsigned) cfg->sleep_seconds);
}
