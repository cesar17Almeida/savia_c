// In-RAM ring storage. The data is tiny (hourly aggregates over >=48 h), so a
// fixed ring is plenty for bring-up; a flash-backed ring (wear-levelled) comes
// later behind the same interface. Pure C -> the query/aggregation paths
// unit-test on the host.
#include "savia/storage.h"
#include <string.h>

#define READINGS_CAP SAVIA_READINGS_CAP
#define PRED_CAP     64

static savia_reading_t s_rd[READINGS_CAP];
static size_t s_rd_count, s_rd_head;

// Set by anything that changes the ring, cleared by storage_take_dirty(). The
// supervisor uses it to skip a flash write when nothing was captured this cycle.
static bool s_dirty;

static savia_prediction_t s_pred[PRED_CAP];
static size_t s_pred_count, s_pred_head;

void storage_init(void) {
    s_rd_count = s_rd_head = 0;
    s_pred_count = s_pred_head = 0;
    s_dirty = false;
}

void storage_clear(void) {
    s_rd_count = s_rd_head = 0;
    s_pred_count = s_pred_head = 0;
    s_dirty = true;   // an emptied ring is a change worth persisting
}

// --- helpers ----------------------------------------------------------------

static size_t ring_start(size_t count, size_t head, size_t cap) {
    return (count < cap) ? 0 : head;   // oldest entry (FIFO order)
}

static bool in_range(uint64_t ts, uint64_t from_ms, uint64_t to_ms) {
    return ts >= from_ms && ts < to_ms;
}

// --- readings ---------------------------------------------------------------

bool storage_append_reading(const savia_reading_t *r) {
    s_rd[s_rd_head] = *r;
    s_rd_head = (s_rd_head + 1) % READINGS_CAP;
    if (s_rd_count < READINGS_CAP) s_rd_count++;
    s_dirty = true;
    return true;
}

size_t storage_rebase_provisional(uint64_t delta_ms) {
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_rd_count; i++) {
        savia_reading_t *e = &s_rd[idx];
        idx = (idx + 1) % READINGS_CAP;
        if (e->ts_ms < SAVIA_TS_PROVISIONAL_MAX) { e->ts_ms += delta_ms; n++; }
    }
    if (n) s_dirty = true;
    return n;
}

bool storage_upsert_reading(const savia_reading_t *r, bool *created) {
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    for (size_t i = 0; i < s_rd_count; i++) {
        savia_reading_t *e = &s_rd[idx];
        if (e->ts_ms == r->ts_ms && e->port == r->port &&
            e->kind == r->kind && e->depth_cm == r->depth_cm) {
            e->value = r->value;            // overwrite in place
            s_dirty = true;
            if (created) *created = false;
            return true;
        }
        idx = (idx + 1) % READINGS_CAP;
    }
    if (created) *created = true;
    return storage_append_reading(r);
}

// Reverse the raw slice [a, b) of the readings array.
static void rd_reverse(size_t a, size_t b) {
    while (a + 1 < b) {
        savia_reading_t t = s_rd[a];
        s_rd[a] = s_rd[--b];
        s_rd[b] = t;
        a++;
    }
}

// Normalise the ring so the oldest entry sits at index 0 (head = count). Only a
// FULL ring can be off-zero, and then the block starts at head; three reversals
// rotate it home in O(n) with no scratch buffer -- 600 readings would be ~10 KB
// of stack we do not have.
static void rd_normalise(void) {
    size_t start = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    if (start == 0) return;
    rd_reverse(0, start);
    rd_reverse(start, READINGS_CAP);
    rd_reverse(0, READINGS_CAP);
    s_rd_head = s_rd_count % READINGS_CAP;
}

size_t storage_clear_port(uint8_t port) {
    if (s_rd_count == 0) return 0;
    rd_normalise();                       // now the block is [0, s_rd_count)
    size_t kept = 0, removed = 0;
    for (size_t i = 0; i < s_rd_count; i++) {
        if (s_rd[i].port == port) { removed++; continue; }
        if (kept != i) s_rd[kept] = s_rd[i];   // dest always trails src
        kept++;
    }
    s_rd_count = kept;
    s_rd_head  = kept % READINGS_CAP;
    if (removed) s_dirty = true;
    return removed;
}

size_t storage_query_raw(uint64_t from_ms, uint64_t to_ms, size_t limit,
                         savia_reading_t *out, size_t out_cap) {
    size_t eff = out_cap;
    if (limit != 0 && limit < eff) eff = limit;
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_rd_count && n < eff; i++) {
        const savia_reading_t *r = &s_rd[idx];
        idx = (idx + 1) % READINGS_CAP;
        if (in_range(r->ts_ms, from_ms, to_ms)) out[n++] = *r;
    }
    return n;
}

size_t storage_count_raw(uint64_t from_ms, uint64_t to_ms) {
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_rd_count; i++) {
        if (in_range(s_rd[idx].ts_ms, from_ms, to_ms)) n++;
        idx = (idx + 1) % READINGS_CAP;
    }
    return n;
}

// Series filter for aggregate(): one (kind, depth) from one port; NULL takes everything.
typedef struct { uint8_t kind, depth_cm, port; } series_filter_t;

static bool series_match(const savia_reading_t *r, uint8_t kind, uint8_t depth_cm) {
    return r->kind == kind && (depth_cm == SAVIA_ANY_DEPTH || r->depth_cm == depth_cm);
}

static size_t aggregate(uint64_t from_ms, uint64_t to_ms, size_t eff,
                        const series_filter_t *f, savia_aggregate_t *out) {
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_rd_count; i++) {
        const savia_reading_t *r = &s_rd[idx];
        idx = (idx + 1) % READINGS_CAP;
        if (!in_range(r->ts_ms, from_ms, to_ms)) continue;
        if (f && (r->port != f->port || !series_match(r, f->kind, f->depth_cm))) continue;
        uint64_t hour = r->ts_ms - (r->ts_ms % 3600000ULL);

        // find an existing bucket (hour, port, kind, depth)
        size_t b = n;
        for (size_t j = 0; j < n; j++) {
            if (out[j].hour_ms == hour && out[j].port == r->port &&
                out[j].kind == r->kind && out[j].depth_cm == r->depth_cm) {
                b = j; break;
            }
        }
        if (b == n) {                       // new bucket
            if (n >= eff) continue;         // no room; drop (rare for mock)
            out[n].hour_ms = hour;
            out[n].port = r->port;
            out[n].kind = r->kind;
            out[n].depth_cm = r->depth_cm;
            out[n].count = 0;
            out[n].mean = 0.0f;             // running sum until finalised
            out[n].min = r->value;
            out[n].max = r->value;
            n++;
        }
        out[b].count++;
        out[b].mean += r->value;            // sum
        if (r->value < out[b].min) out[b].min = r->value;
        if (r->value > out[b].max) out[b].max = r->value;
    }
    for (size_t j = 0; j < n; j++) {        // sum -> mean
        if (out[j].count) out[j].mean /= (float) out[j].count;
    }
    return n;
}

size_t storage_aggregate_hourly(uint64_t from_ms, uint64_t to_ms, size_t limit,
                                savia_aggregate_t *out, size_t out_cap) {
    size_t eff = out_cap;
    if (limit != 0 && limit < eff) eff = limit;
    return aggregate(from_ms, to_ms, eff, NULL, out);
}

size_t storage_aggregate_series(uint64_t from_ms, uint64_t to_ms, uint8_t kind,
                                uint8_t depth_cm, savia_aggregate_t *out, size_t out_cap) {
    int port = -1;                          // lowest port carrying the series in range
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    for (size_t i = 0; i < s_rd_count; i++) {
        const savia_reading_t *r = &s_rd[idx];
        idx = (idx + 1) % READINGS_CAP;
        if (in_range(r->ts_ms, from_ms, to_ms) && series_match(r, kind, depth_cm) &&
            (port < 0 || r->port < port)) port = r->port;
    }
    if (port < 0) return 0;
    series_filter_t f = { kind, depth_cm, (uint8_t) port };
    return aggregate(from_ms, to_ms, out_cap, &f, out);
}

// --- persistence (SDK-free halves; the flash side is storage_store.c) --------

bool storage_take_dirty(void) {
    bool d = s_dirty;
    s_dirty = false;
    return d;
}

size_t storage_reading_count(void) { return s_rd_count; }

const savia_reading_t *storage_reading_at(size_t i) {
    if (i >= s_rd_count) return NULL;
    size_t idx = (ring_start(s_rd_count, s_rd_head, READINGS_CAP) + i) % READINGS_CAP;
    return &s_rd[idx];
}

static void wr_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t) v; p[1] = (uint8_t) (v >> 8);
}
static uint16_t rd_u16_le(const uint8_t *p) {
    return (uint16_t) ((uint16_t) p[0] | ((uint16_t) p[1] << 8));
}
static void wr_u64_le(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t) (v >> (8 * i));
}
static uint64_t rd_u64_le(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t) p[i] << (8 * i);
    return v;
}
// float <-> u32 by bit pattern, via memcpy rather than a cast so this stays free
// of strict-aliasing undefined behaviour. Both ends are little-endian.
static void wr_f32_le(uint8_t *p, float f) {
    uint32_t b; memcpy(&b, &f, sizeof b);
    for (int i = 0; i < 4; i++) p[i] = (uint8_t) (b >> (8 * i));
}
static float rd_f32_le(const uint8_t *p) {
    uint32_t b = 0;
    for (int i = 0; i < 4; i++) b |= (uint32_t) p[i] << (8 * i);
    float f; memcpy(&f, &b, sizeof f);
    return f;
}

void storage_encode_reading(const savia_reading_t *r, uint8_t *out) {
    wr_u64_le(out, r->ts_ms);
    out[8]  = r->port;
    out[9]  = r->depth_cm;
    out[10] = r->kind;
    wr_f32_le(out + 11, r->value);
}

size_t storage_serialize(uint8_t *out, size_t cap) {
    size_t need = 2 + s_rd_count * SAVIA_READING_WIRE_BYTES;
    if (!out || cap < need) return 0;
    wr_u16_le(out, (uint16_t) s_rd_count);
    for (size_t i = 0; i < s_rd_count; i++)
        storage_encode_reading(storage_reading_at(i), out + 2 + i * SAVIA_READING_WIRE_BYTES);
    return need;
}

size_t storage_restore(const uint8_t *data, size_t len) {
    if (!data || len < 2) return 0;
    uint16_t n = rd_u16_le(data);
    // Refuse rather than load a prefix: a half-restored ring reads as a real
    // history with a hole in it, which is worse than starting empty.
    if (n > READINGS_CAP) return 0;
    if (len < (size_t) 2 + (size_t) n * SAVIA_READING_WIRE_BYTES) return 0;
    size_t kept = 0;
    for (uint16_t i = 0; i < n; i++) {
        const uint8_t *p = data + 2 + (size_t) i * SAVIA_READING_WIRE_BYTES;
        uint64_t ts = rd_u64_le(p);
        // An uptime stamp from an earlier power cycle can never be rebased correctly.
        if (ts < SAVIA_TS_PROVISIONAL_MAX) continue;
        s_rd[kept].ts_ms    = ts;
        s_rd[kept].port     = p[8];
        s_rd[kept].depth_cm = p[9];
        s_rd[kept].kind     = p[10];
        s_rd[kept].value    = rd_f32_le(p + 11);
        kept++;
    }
    s_rd_count = kept;
    s_rd_head  = kept % READINGS_CAP;
    s_dirty    = kept != n;  // identical to flash unless provisional rows were dropped
    return kept;
}

// --- predictions ------------------------------------------------------------


bool storage_append_prediction(const savia_prediction_t *p) {
    s_pred[s_pred_head] = *p;
    s_pred_head = (s_pred_head + 1) % PRED_CAP;
    if (s_pred_count < PRED_CAP) s_pred_count++;
    return true;
}

void storage_clear_predictions(void) {
    s_pred_count = s_pred_head = 0;
}

size_t storage_query_pred(uint64_t from_ms, uint64_t to_ms, size_t limit,
                          savia_prediction_t *out, size_t out_cap) {
    size_t eff = out_cap;
    if (limit != 0 && limit < eff) eff = limit;
    size_t idx = ring_start(s_pred_count, s_pred_head, PRED_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_pred_count && n < eff; i++) {
        const savia_prediction_t *p = &s_pred[idx];
        idx = (idx + 1) % PRED_CAP;
        if (in_range(p->ts_ms, from_ms, to_ms)) out[n++] = *p;
    }
    return n;
}

size_t storage_count_pred(uint64_t from_ms, uint64_t to_ms) {
    size_t idx = ring_start(s_pred_count, s_pred_head, PRED_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_pred_count; i++) {
        if (in_range(s_pred[idx].ts_ms, from_ms, to_ms)) n++;
        idx = (idx + 1) % PRED_CAP;
    }
    return n;
}
