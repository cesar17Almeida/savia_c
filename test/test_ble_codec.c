// Host test for the BLE wire codec (no Pico SDK, no hardware). Produces CBOR
// that the Python cross-check feeds through savia_py's OWN codec, proving
// TerraLink-compatible output. See test/crosscheck_ble_codec.py.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "savia/ble_codec.h"
#include "savia/types.h"

static size_t read_file(const char *path, uint8_t *buf, size_t cap) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }
    size_t n = fread(buf, 1, cap, f);
    fclose(f);
    return n;
}

static void write_file(const char *path, const uint8_t *buf, size_t n) {
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return; }
    fwrite(buf, 1, n, f);
    fclose(f);
}

// Collect each frame to a file as [u16 big-endian len][frame bytes].
static void emit_frame(const uint8_t *frame, size_t len, void *ctx) {
    FILE *f = (FILE *) ctx;
    uint8_t hdr[2] = { (uint8_t)(len >> 8), (uint8_t)(len & 0xff) };
    fwrite(hdr, 1, 2, f);
    fwrite(frame, 1, len, f);
}

int main(void) {
    // 1) Parse a data_request that Python (cbor2) generated.
    uint8_t req[256];
    size_t rn = read_file("/tmp/savia_req.cbor", req, sizeof(req));
    assert(rn > 0);
    ble_data_request_t dr;
    assert(ble_parse_data_request(req, rn, &dr));
    assert(dr.version == 1);
    assert(strcmp(dr.op, "get") == 0);
    assert(strcmp(dr.kind, "raw") == 0);
    assert(dr.has_from && dr.from_ms == 1000);
    assert(dr.has_to && dr.to_ms == 2000);
    assert(dr.has_limit && dr.limit == 50);
    printf("test_ble_codec: request parse OK (op=%s kind=%s from=%llu to=%llu limit=%llu)\n",
           dr.op, dr.kind, (unsigned long long) dr.from_ms,
           (unsigned long long) dr.to_ms, (unsigned long long) dr.limit);

    // 2) Serialize 2 mock readings (what the Pico's sensor stub produces).
    savia_reading_t rows[2] = {
        { .ts_ms = 1700000000000ULL, .port = 1, .depth_cm = 10,
          .kind = READING_SOIL_MOISTURE, .value = 0.75f },
        { .ts_ms = 1700000000000ULL, .port = 1, .depth_cm = 30,
          .kind = READING_SOIL_MOISTURE, .value = 0.77f },
    };
    uint8_t payload[512];
    size_t plen = ble_serialize_readings(rows, 2, payload, sizeof(payload));
    assert(plen > 0);
    write_file("/tmp/savia_readings.cbor", payload, plen);
    printf("test_ble_codec: serialized 2 readings -> %zu CBOR bytes\n", plen);

    // 3) Chunk it small (chunk_size=8) to exercise multi-frame reassembly.
    FILE *ff = fopen("/tmp/savia_frames.bin", "wb");
    assert(ff);
    ble_chunk_encode(payload, plen, 8, emit_frame, ff);
    fclose(ff);
    printf("test_ble_codec: chunked -> /tmp/savia_frames.bin\n");

    printf("test_ble_codec: OK\n");
    return 0;
}
