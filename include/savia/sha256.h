// Minimal one-shot SHA-256 (public-domain algorithm, SDK-free, host-testable).
// Used for the BLE auth proof: proof = SHA256(auth_key || nonce).
#ifndef SAVIA_SHA256_H
#define SAVIA_SHA256_H

#include <stdint.h>
#include <stddef.h>

#define SAVIA_SHA256_LEN 32

void savia_sha256(const uint8_t *data, size_t len, uint8_t out[SAVIA_SHA256_LEN]);

#endif // SAVIA_SHA256_H
