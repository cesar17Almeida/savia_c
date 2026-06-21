#include "savia/auth.h"
#include "savia/sha256.h"
#include <string.h>

void auth_compute_proof(const uint8_t key[SAVIA_AUTH_KEY_LEN],
                        const uint8_t nonce[SAVIA_AUTH_NONCE_LEN],
                        uint8_t out[SAVIA_AUTH_PROOF_LEN]) {
    uint8_t buf[SAVIA_AUTH_KEY_LEN + SAVIA_AUTH_NONCE_LEN];
    memcpy(buf, key, SAVIA_AUTH_KEY_LEN);
    memcpy(buf + SAVIA_AUTH_KEY_LEN, nonce, SAVIA_AUTH_NONCE_LEN);
    savia_sha256(buf, sizeof(buf), out);
}

bool auth_key_is_set(const uint8_t key[SAVIA_AUTH_KEY_LEN]) {
    uint8_t acc = 0;
    for (int i = 0; i < SAVIA_AUTH_KEY_LEN; i++) acc |= key[i];
    return acc != 0;
}

bool auth_proof_equal(const uint8_t a[SAVIA_AUTH_PROOF_LEN], const uint8_t b[SAVIA_AUTH_PROOF_LEN]) {
    uint8_t diff = 0;
    for (int i = 0; i < SAVIA_AUTH_PROOF_LEN; i++) diff |= (uint8_t)(a[i] ^ b[i]);
    return diff == 0;
}
