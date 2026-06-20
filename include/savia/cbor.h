// Minimal CBOR writer + reader (pure C, no deps) -- just the subset Savia needs
// to mirror savia_py's cbor2 output: uints, text, doubles, bools, byte-strings,
// definite-length arrays/maps. SDK-free so it unit-tests on the host.
#ifndef SAVIA_CBOR_H
#define SAVIA_CBOR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// --- Writer -----------------------------------------------------------------
typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   len;       // bytes written
    bool     overflow;  // set if a write didn't fit (output then invalid)
} cbor_writer_t;

void cbor_w_init(cbor_writer_t *w, uint8_t *buf, size_t cap);
void cbor_w_array(cbor_writer_t *w, uint64_t n);   // definite-length array header
void cbor_w_map(cbor_writer_t *w, uint64_t n);     // definite-length map header
void cbor_w_uint(cbor_writer_t *w, uint64_t v);
void cbor_w_int(cbor_writer_t *w, int64_t v);
void cbor_w_text(cbor_writer_t *w, const char *s, size_t n);
void cbor_w_textz(cbor_writer_t *w, const char *s);   // null-terminated
void cbor_w_bytes(cbor_writer_t *w, const uint8_t *p, size_t n);
void cbor_w_double(cbor_writer_t *w, double v);
void cbor_w_bool(cbor_writer_t *w, bool b);

// --- Reader (minimal: enough to parse a flat control-message map) -----------
typedef struct {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;
    bool           err;
} cbor_reader_t;

void cbor_r_init(cbor_reader_t *r, const uint8_t *buf, size_t len);
bool cbor_r_map(cbor_reader_t *r, uint64_t *count);     // read map header
bool cbor_r_text(cbor_reader_t *r, const char **s, size_t *n);  // ptr into buf
bool cbor_r_uint(cbor_reader_t *r, uint64_t *v);
bool cbor_r_skip(cbor_reader_t *r);   // skip any one value (incl. nested)

// Convenience: does the just-read text equal `lit`?
bool cbor_text_eq(const char *s, size_t n, const char *lit);

#endif // SAVIA_CBOR_H
