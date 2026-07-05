// In-RAM ring storage. The data is tiny (hourly aggregates over >=48 h), so a
// fixed ring is plenty for bring-up; a flash-backed ring (wear-levelled) comes
// later behind the same interface. Pure C -> the query/aggregation paths
// unit-test on the host.
#include "savia/storage.h"
#include <string.h>

#define READINGS_CAP 600
#define PRED_CAP     64

static savia_reading_t s_rd[READINGS_CAP];
static size_t s_rd_count, s_rd_head;

static savia_prediction_t s_pred[PRED_CAP];
static size_t s_pred_count, s_pred_head;

void storage_init(void) {
    s_rd_count = s_rd_head = 0;
    s_pred_count = s_pred_head = 0;
}

void storage_clear(void) {
    s_rd_count = s_rd_head = 0;
    s_pred_count = s_pred_head = 0;
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
    return n;
}

bool storage_upsert_reading(const savia_reading_t *r, bool *created) {
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    for (size_t i = 0; i < s_rd_count; i++) {
        savia_reading_t *e = &s_rd[idx];
        if (e->ts_ms == r->ts_ms && e->port == r->port &&
            e->kind == r->kind && e->depth_cm == r->depth_cm) {
            e->value = r->value;            // overwrite in place
            if (created) *created = false;
            return true;
        }
        idx = (idx + 1) % READINGS_CAP;
    }
    if (created) *created = true;
    return storage_append_reading(r);
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

size_t storage_aggregate_hourly(uint64_t from_ms, uint64_t to_ms, size_t limit,
                                savia_aggregate_t *out, size_t out_cap) {
    size_t eff = out_cap;
    if (limit != 0 && limit < eff) eff = limit;
    size_t idx = ring_start(s_rd_count, s_rd_head, READINGS_CAP);
    size_t n = 0;
    for (size_t i = 0; i < s_rd_count; i++) {
        const savia_reading_t *r = &s_rd[idx];
        idx = (idx + 1) % READINGS_CAP;
        if (!in_range(r->ts_ms, from_ms, to_ms)) continue;
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
