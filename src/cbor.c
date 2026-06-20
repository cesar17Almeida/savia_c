#include "savia/cbor.h"
#include <string.h>

// --- Writer -----------------------------------------------------------------

void cbor_w_init(cbor_writer_t *w, uint8_t *buf, size_t cap) {
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->overflow = false;
}

static void w_byte(cbor_writer_t *w, uint8_t b) {
    if (w->len < w->cap) {
        w->buf[w->len] = b;
    } else {
        w->overflow = true;
    }
    w->len++;
}

// Write a CBOR head: major type (already shifted to bits 5-7) + argument,
// using the shortest of immediate / 1 / 2 / 4 / 8 bytes (big-endian).
static void w_head(cbor_writer_t *w, uint8_t major, uint64_t val) {
    if (val < 24) {
        w_byte(w, major | (uint8_t) val);
    } else if (val < 0x100ULL) {
        w_byte(w, major | 24);
        w_byte(w, (uint8_t) val);
    } else if (val < 0x10000ULL) {
        w_byte(w, major | 25);
        w_byte(w, (uint8_t)(val >> 8));
        w_byte(w, (uint8_t) val);
    } else if (val < 0x100000000ULL) {
        w_byte(w, major | 26);
        for (int s = 24; s >= 0; s -= 8) w_byte(w, (uint8_t)(val >> s));
    } else {
        w_byte(w, major | 27);
        for (int s = 56; s >= 0; s -= 8) w_byte(w, (uint8_t)(val >> s));
    }
}

void cbor_w_array(cbor_writer_t *w, uint64_t n) { w_head(w, 0x80, n); }
void cbor_w_map(cbor_writer_t *w, uint64_t n)   { w_head(w, 0xa0, n); }
void cbor_w_uint(cbor_writer_t *w, uint64_t v)  { w_head(w, 0x00, v); }

void cbor_w_int(cbor_writer_t *w, int64_t v) {
    if (v >= 0) w_head(w, 0x00, (uint64_t) v);
    else        w_head(w, 0x20, (uint64_t)(-1 - v));   // negative: major 1
}

void cbor_w_text(cbor_writer_t *w, const char *s, size_t n) {
    w_head(w, 0x60, (uint64_t) n);
    for (size_t i = 0; i < n; i++) w_byte(w, (uint8_t) s[i]);
}

void cbor_w_textz(cbor_writer_t *w, const char *s) { cbor_w_text(w, s, strlen(s)); }

void cbor_w_bytes(cbor_writer_t *w, const uint8_t *p, size_t n) {
    w_head(w, 0x40, (uint64_t) n);
    for (size_t i = 0; i < n; i++) w_byte(w, p[i]);
}

void cbor_w_double(cbor_writer_t *w, double v) {
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));      // IEEE-754 double, host order
    w_byte(w, 0xfb);                       // major 7, double-precision float
    for (int s = 56; s >= 0; s -= 8) w_byte(w, (uint8_t)(bits >> s));  // big-endian
}

void cbor_w_bool(cbor_writer_t *w, bool b) { w_byte(w, b ? 0xf5 : 0xf4); }

void cbor_w_null(cbor_writer_t *w) { w_byte(w, 0xf6); }   // major 7, simple 22

// --- Reader -----------------------------------------------------------------

void cbor_r_init(cbor_reader_t *r, const uint8_t *buf, size_t len) {
    r->buf = buf;
    r->len = len;
    r->pos = 0;
    r->err = false;
}

// Read a head: returns major (0-7) and the decoded argument.
static bool r_head(cbor_reader_t *r, uint8_t *major, uint64_t *val) {
    if (r->err || r->pos >= r->len) { r->err = true; return false; }
    uint8_t ib = r->buf[r->pos++];
    *major = ib >> 5;
    uint8_t ai = ib & 0x1f;
    if (ai < 24) { *val = ai; return true; }
    int nbytes = (ai == 24) ? 1 : (ai == 25) ? 2 : (ai == 26) ? 4 : (ai == 27) ? 8 : -1;
    if (nbytes < 0 || r->pos + (size_t) nbytes > r->len) { r->err = true; return false; }
    uint64_t v = 0;
    for (int i = 0; i < nbytes; i++) v = (v << 8) | r->buf[r->pos++];
    *val = v;
    return true;
}

bool cbor_r_map(cbor_reader_t *r, uint64_t *count) {
    uint8_t major; uint64_t val;
    if (!r_head(r, &major, &val) || major != 5) { r->err = true; return false; }
    *count = val;
    return true;
}

bool cbor_r_text(cbor_reader_t *r, const char **s, size_t *n) {
    uint8_t major; uint64_t val;
    if (!r_head(r, &major, &val) || major != 3) { r->err = true; return false; }
    if (r->pos + val > r->len) { r->err = true; return false; }
    *s = (const char *) (r->buf + r->pos);
    *n = (size_t) val;
    r->pos += val;
    return true;
}

bool cbor_r_uint(cbor_reader_t *r, uint64_t *v) {
    uint8_t major; uint64_t val;
    if (!r_head(r, &major, &val) || major != 0) { r->err = true; return false; }
    *v = val;
    return true;
}

bool cbor_r_skip(cbor_reader_t *r) {
    uint8_t major; uint64_t val;
    if (!r_head(r, &major, &val)) return false;
    switch (major) {
        case 0: case 1: case 7:        // uint / negint / simple+float
            if (major == 7) {          // float payload already consumed via ai bytes?
                // ai 25/26/27 floats: r_head consumed their bytes already; 20-23 simple: none.
            }
            return !r->err;
        case 2: case 3:                // bytes / text: skip `val` bytes
            if (r->pos + val > r->len) { r->err = true; return false; }
            r->pos += val;
            return true;
        case 4:                        // array: skip `val` items
            for (uint64_t i = 0; i < val; i++) if (!cbor_r_skip(r)) return false;
            return true;
        case 5:                        // map: skip `val` key+value pairs
            for (uint64_t i = 0; i < 2 * val; i++) if (!cbor_r_skip(r)) return false;
            return true;
        default:
            r->err = true;
            return false;
    }
}

bool cbor_text_eq(const char *s, size_t n, const char *lit) {
    return strlen(lit) == n && memcmp(s, lit, n) == 0;
}
