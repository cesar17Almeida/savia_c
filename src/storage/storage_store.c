// Flash image of the measurement state, so a reset does not cost 24 h of
// re-sampling before the LSTM can infer again.
//
// WHAT IS STORED: the whole readings ring plus the TA window, in one record.
// Both are needed to infer and losing either costs the same, so they are written
// and read together.
//
// WHY AN IMAGE AND NOT A LOG: the ring supports operations a NOR append-only log
// cannot express -- storage_upsert_reading overwrites a value in place (that is
// the BLE `ingest` path), storage_rebase_provisional rewrites every provisional
// timestamp after the first clock sync, and storage_clear_port deletes and
// compacts. Persisting an image keeps ALL of that working untouched: flash is a
// periodic photograph of RAM, never the source of truth.
//
// A/B BANKS: the record is written to whichever bank is NOT current, and the
// newest valid `seq` wins on load. A power cut mid-write therefore costs the new
// image, never the old one -- unlike config_store/clock_store, which erase the
// only copy they have before programming it.
//
// WEAR: two banks x 3 sectors, one bank erased per save. At a save per capture
// cycle (hourly) each sector sees ~12 erases a day, so ~22 years against a
// 100k-cycle endurance. Not the constraint here.
//
// COST: ~150 ms of paused BLE per save (3 sector erases dominate), which is why
// the supervisor calls this just before the nap, after ble_poll has returned.
// The figure is a datasheet estimate for this class of NOR -- worth measuring on
// the board if a save is ever moved somewhere a phone could be mid-transfer.
//
// IF THIS EVER BECOMES A PROBLEM (the "option C" fallback): persist only the
// CLOSED HOURLY AGGREGATES instead of raw readings. 48 h x 2 series is 96
// aggregates ~ 2.7 KB, one sector instead of three, and a closed hour is
// immutable by construction -- so the upsert/rebase/clear_port objection above
// disappears and the write is ~3x cheaper. What it gives up is the raw readings
// the phone has not synced yet, which would still die with the power. The hook
// would be "an hour just closed" in the supervisor, plus letting
// lstm_gather_inputs merge persisted aggregates with whatever is in RAM.
#include "savia/storage_store.h"
#include "savia/storage.h"
#include "savia/weather.h"
#include "savia/log.h"
#include <string.h>
#include "hardware/flash.h"
#include "pico/flash.h"

#define STORE_MAGIC   0x53565244u    // 'SVRD'
#define STORE_VERSION 1u

// Three 4 KB sectors per bank; two banks sit below clock_store's sector, which
// itself sits below config_store's. Flash is 4 MB and the binary uses ~1.3 MB of
// it from the bottom, so these 32 KB at the top are free by a wide margin.
#define STORE_BANK_SECTORS 3
#define STORE_BANK_BYTES   (STORE_BANK_SECTORS * FLASH_SECTOR_SIZE)
#define STORE_BANK_A       (PICO_FLASH_SIZE_BYTES - 5 * FLASH_SECTOR_SIZE)
#define STORE_BANK_B       (PICO_FLASH_SIZE_BYTES - 8 * FLASH_SECTOR_SIZE)

// Header: magic(4) version(2) wire(2) seq(4) rd_len(2) wx_len(2).
#define STORE_HDR_BYTES 16
#define STORE_MAX_BYTES (STORE_HDR_BYTES + SAVIA_STORAGE_BLOB_MAX + WEATHER_BLOB_MAX + 4)

_Static_assert(STORE_MAX_BYTES <= STORE_BANK_BYTES, "state record exceeds one bank");
_Static_assert(STORE_BANK_A + STORE_BANK_BYTES <= PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE,
               "bank A overlaps the clock/config sectors");
_Static_assert(STORE_BANK_B + STORE_BANK_BYTES <= STORE_BANK_A, "bank B overlaps bank A");

// Which bank holds the newest image, and its sequence number. Both are learned
// at load; before that the first save simply writes bank A.
static uint32_t s_seq;
static uint32_t s_bank = STORE_BANK_B;   // so the first save lands in A

static uint32_t crc32_update(uint32_t c, uint8_t b) {
    c ^= b;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & -(c & 1));
    return c;
}

static uint32_t crc32_buf(uint32_t c, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) c = crc32_update(c, p[i]);
    return c;
}

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t) (p[0] | (p[1] << 8)); }
static uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t) p[0] | ((uint32_t) p[1] << 8) |
           ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

/** Validate one bank in place (flash is memory-mapped, so nothing is copied) and
 *  report its sequence number and section offsets. */
static bool bank_parse(uint32_t base, uint32_t *seq, const uint8_t **rd, uint16_t *rd_len,
                       const uint8_t **wx, uint16_t *wx_len) {
    const uint8_t *p = (const uint8_t *) (XIP_BASE + base);
    if (rd_u32(p) != STORE_MAGIC) return false;
    if (rd_u16(p + 4) != STORE_VERSION) return false;
    // A firmware whose reading layout changed must not read an old image as if it
    // were its own; the record is dropped and the station starts empty.
    if (rd_u16(p + 6) != SAVIA_READING_WIRE_BYTES) return false;
    uint16_t rl = rd_u16(p + 12), wl = rd_u16(p + 14);
    if (rl > SAVIA_STORAGE_BLOB_MAX || wl > WEATHER_BLOB_MAX) return false;
    size_t body = STORE_HDR_BYTES + (size_t) rl + (size_t) wl;
    if (body + 4 > STORE_BANK_BYTES) return false;
    if (crc32_buf(0xffffffffu, p, body) != ~rd_u32(p + body)) return false;
    *seq = rd_u32(p + 8);
    *rd = p + STORE_HDR_BYTES;   *rd_len = rl;
    *wx = p + STORE_HDR_BYTES + rl; *wx_len = wl;
    return true;
}

// The TA section of the image loaded at boot, kept for storage_store_adopt_weather.
// Points into memory-mapped flash and stays valid until that bank is erased --
// which cannot happen before adoption, since a save always writes the other bank.
static const uint8_t *s_wx;
static uint16_t       s_wx_len;

size_t storage_store_load(void) {
    const uint8_t *rd = NULL, *wx = NULL;
    uint16_t rd_len = 0, wx_len = 0;
    uint32_t seq = 0;
    bool found = false;

    const uint32_t banks[2] = { STORE_BANK_A, STORE_BANK_B };
    for (int i = 0; i < 2; i++) {
        const uint8_t *r, *w;
        uint16_t rl, wl;
        uint32_t s;
        if (!bank_parse(banks[i], &s, &r, &rl, &w, &wl)) continue;
        if (found && s <= seq) continue;
        seq = s; rd = r; rd_len = rl; wx = w; wx_len = wl;
        s_bank = banks[i];
        found = true;
    }
    if (!found) {
        LOG_INFO("storage_store: no image in flash, starting empty\n");
        return 0;
    }
    s_seq = seq;
    s_wx = wx;
    s_wx_len = wx_len;

    size_t n = storage_restore(rd, rd_len);
    LOG_INFO("storage_store: restored %u readings (seq=%u)\n", (unsigned) n, (unsigned) seq);
    return n;
}

bool storage_store_adopt_weather(uint64_t now_ms) {
    if (!s_wx_len) return false;
    // A downlink during bring-up beats anything stored: it is current by
    // construction, and overwriting it with an older window would be a downgrade.
    if (weather_is_set()) {
        LOG_INFO("storage_store: TA window already fresh from a downlink, image ignored\n");
        return false;
    }
    bool ok = weather_restore(s_wx, s_wx_len, now_ms, WEATHER_RESTORE_MAX_AGE_MS);
    LOG_INFO("storage_store: stored TA window %s\n",
             ok ? "adopted" : "dropped (too old for this hour)");
    return ok;
}

// --- save -------------------------------------------------------------------

// Streamed page by page rather than staged whole: the record is ~9.3 KB and a
// static buffer that size would be 10% of the RAM this board has left.
typedef struct {
    uint32_t base;                    // flash offset of the bank being written
    uint32_t written;                 // bytes emitted
    uint32_t crc;                     // running CRC over everything but itself
    uint8_t  page[FLASH_PAGE_SIZE];   // FLASH PROGRAM granularity is one page
} sink_t;

static sink_t s_sink;

static void sink_flush_page(sink_t *k) {
    flash_range_program(k->base + (k->written - FLASH_PAGE_SIZE), k->page, FLASH_PAGE_SIZE);
}

static void sink_put(sink_t *k, uint8_t b, bool in_crc) {
    if (in_crc) k->crc = crc32_update(k->crc, b);
    k->page[k->written % FLASH_PAGE_SIZE] = b;
    k->written++;
    if (k->written % FLASH_PAGE_SIZE == 0) sink_flush_page(k);
}

static void sink_put_buf(sink_t *k, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) sink_put(k, p[i], true);
}

static void sink_put_u16(sink_t *k, uint16_t v) {
    sink_put(k, (uint8_t) v, true); sink_put(k, (uint8_t) (v >> 8), true);
}
static void sink_put_u32(sink_t *k, uint32_t v, bool in_crc) {
    for (int i = 0; i < 4; i++) sink_put(k, (uint8_t) (v >> (8 * i)), in_crc);
}

/**
 * Pad the tail of the last page with 0xff (the erased state) and program it.
 *
 * Returns immediately when the record happened to end exactly on a page
 * boundary: sink_put already programmed that page, and programming a page twice
 * between erases is not something NOR flash is guaranteed to survive cleanly.
 */
static void sink_finish(sink_t *k) {
    if (k->written % FLASH_PAGE_SIZE == 0) return;
    while (k->written % FLASH_PAGE_SIZE) {
        k->page[k->written % FLASH_PAGE_SIZE] = 0xff;
        k->written++;
    }
    sink_flush_page(k);
}

// Runs inside flash_safe_execute: the cyw43/BTstack context is held off, so the
// ring cannot change under us and no snapshot is needed (unlike clock_store,
// which serializes outside and passes a blob in). No logging here.
static void do_save(void *param) {
    (void) param;
    // Serializing the TA window first is what makes its length known before the
    // header goes out; at ~300 B it is cheap to stage.
    static uint8_t wx[WEATHER_BLOB_MAX];
    size_t wx_len = weather_serialize(wx, sizeof wx);

    size_t count = storage_reading_count();
    uint16_t rd_len = (uint16_t) (2 + count * SAVIA_READING_WIRE_BYTES);

    flash_range_erase(s_sink.base, STORE_BANK_BYTES);

    s_sink.written = 0;
    s_sink.crc = 0xffffffffu;
    sink_put_u32(&s_sink, STORE_MAGIC, true);
    sink_put_u16(&s_sink, STORE_VERSION);
    sink_put_u16(&s_sink, SAVIA_READING_WIRE_BYTES);
    sink_put_u32(&s_sink, s_seq, true);
    sink_put_u16(&s_sink, rd_len);
    sink_put_u16(&s_sink, (uint16_t) wx_len);

    sink_put_u16(&s_sink, (uint16_t) count);
    uint8_t enc[SAVIA_READING_WIRE_BYTES];
    for (size_t i = 0; i < count; i++) {
        storage_encode_reading(storage_reading_at(i), enc);
        sink_put_buf(&s_sink, enc, sizeof enc);
    }
    sink_put_buf(&s_sink, wx, wx_len);

    sink_put_u32(&s_sink, ~s_sink.crc, false);   // the CRC is not part of itself
    sink_finish(&s_sink);
}

void storage_store_save(void) {
    // Always write the bank we are NOT currently reading from, so the previous
    // image survives a power cut in the middle of this one.
    s_sink.base = (s_bank == STORE_BANK_A) ? STORE_BANK_B : STORE_BANK_A;
    s_seq++;
    int rc = flash_safe_execute(do_save, NULL, 2000);
    if (rc != PICO_OK) {
        s_seq--;                       // the image never landed; do not claim it did
        LOG_WARN("storage_store: flash save rc=%d\n", rc);
        return;
    }
    s_bank = s_sink.base;
    LOG_INFO("storage_store: saved %u readings (seq=%u)\n",
             (unsigned) storage_reading_count(), (unsigned) s_seq);
}
