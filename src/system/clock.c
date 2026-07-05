#include "savia/clock.h"

// --- raw running clock ------------------------------------------------------

static uint64_t s_epoch_base_ms;   // epoch_ms - uptime_ms at last sync
static uint64_t s_last_sync_ms;
static bool     s_set;

// --- authoritative-sync ring (newest at [0]) --------------------------------

static clock_sample_t s_ring[CLOCK_RING_MAX];
static uint8_t        s_count;
static bool           s_dirty;            // ring changed since last persist
static bool           s_first_sync_done;  // first accepted sync THIS power cycle
static uint64_t       s_boot_outage_ms;   // gap measured at that first sync

void clock_set(uint64_t epoch_ms, uint64_t uptime_ms) {
    s_epoch_base_ms = epoch_ms - uptime_ms;
    s_last_sync_ms = epoch_ms;
    s_set = true;
}

uint64_t clock_now(uint64_t uptime_ms) {
    if (!s_set) return 0;
    return s_epoch_base_ms + uptime_ms;
}

bool clock_is_set(void) { return s_set; }

uint64_t clock_last_sync_ms(void) { return s_last_sync_ms; }

// --- ring helpers -----------------------------------------------------------

static void ring_push(uint64_t epoch_ms, uint64_t uptime_ms, uint8_t source) {
    uint8_t n = s_count < CLOCK_RING_MAX ? (uint8_t)(s_count + 1) : CLOCK_RING_MAX;
    for (int i = n - 1; i > 0; i--) s_ring[i] = s_ring[i - 1];
    s_ring[0].epoch_ms = epoch_ms;
    s_ring[0].uptime_ms = uptime_ms;
    s_ring[0].source = source;
    s_count = n;
}

uint64_t clock_last_known(void) { return s_count ? s_ring[0].epoch_ms : 0; }

uint64_t clock_boot_outage_ms(void) { return s_boot_outage_ms; }

uint8_t clock_get_ring(clock_sample_t *out, uint8_t max) {
    uint8_t n = s_count < max ? s_count : max;
    for (uint8_t i = 0; i < n; i++) out[i] = s_ring[i];
    return n;
}

bool clock_apply_sync(uint64_t epoch_ms, uint64_t uptime_ms, clock_source_t source,
                      uint64_t *outage_ms) {
    if (outage_ms) *outage_ms = 0;

    // Absolute plausibility: a garbage downlink (0, 0xFFFFFFFF, ...) never lands.
    if (epoch_ms < CLOCK_EPOCH_MIN_MS || epoch_ms >= CLOCK_EPOCH_MAX_MS) return false;

    // Monotonic vs the last known-good, tolerating small cross-source jitter. Real
    // time only moves forward; a larger backward jump is a bad reading -> reject.
    uint64_t lk = clock_last_known();
    if (lk != 0 && epoch_ms + CLOCK_BACKWARD_SLACK_MS < lk) return false;

    // Gap vs the previous known-good. On the first sync after a reboot this equals
    // the power-off duration (lk is the pre-outage reference seeded from flash).
    uint64_t gap = (lk != 0 && epoch_ms > lk) ? (epoch_ms - lk) : 0;
    if (!s_first_sync_done) {
        s_boot_outage_ms = gap;
        s_first_sync_done = true;
    }
    if (outage_ms) *outage_ms = gap;

    ring_push(epoch_ms, uptime_ms, (uint8_t) source);
    clock_set(epoch_ms, uptime_ms);
    s_dirty = true;
    return true;
}

// --- persistence bridge (little-endian; host and RP2xxx are both LE) ---------

static void wr_u64_le(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i));
}

static uint64_t rd_u64_le(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t) p[i] << (8 * i);
    return v;
}

size_t clock_serialize_ring(uint8_t *out, size_t cap) {
    size_t need = 1 + (size_t) s_count * 9;
    if (cap < need) return 0;
    size_t p = 0;
    out[p++] = s_count;
    for (uint8_t i = 0; i < s_count; i++) {
        wr_u64_le(out + p, s_ring[i].epoch_ms); p += 8;
        out[p++] = s_ring[i].source;
    }
    return p;
}

bool clock_seed_ring(const uint8_t *data, size_t len) {
    s_count = 0;
    if (!data || len < 1) return false;
    uint8_t n = data[0];
    if (n > CLOCK_RING_MAX) return false;
    if (len < (size_t) 1 + (size_t) n * 9) return false;
    size_t p = 1;
    for (uint8_t i = 0; i < n; i++) {
        uint64_t epoch = rd_u64_le(data + p); p += 8;
        uint8_t  src   = data[p++];
        if (epoch < CLOCK_EPOCH_MIN_MS || epoch >= CLOCK_EPOCH_MAX_MS) return false;  // drop bogus record
        s_ring[i].epoch_ms = epoch;
        s_ring[i].uptime_ms = 0;   // meaningless across a reboot
        s_ring[i].source = src;
    }
    s_count = n;
    return true;
}

bool clock_take_ring_dirty(void) {
    bool d = s_dirty;
    s_dirty = false;
    return d;
}
