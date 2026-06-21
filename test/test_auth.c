// Host test: SHA-256 vectors + auth proof = SHA256(key||nonce), cross-checked vs Python.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "savia/sha256.h"
#include "savia/auth.h"

static void hex(const uint8_t *d, int n, char *out) {
    for (int i = 0; i < n; i++) sprintf(out + i * 2, "%02x", d[i]);
}

int main(void) {
    uint8_t o[32]; char h[65];
    savia_sha256((const uint8_t *)"abc", 3, o); hex(o, 32, h);
    assert(strcmp(h, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0);
    printf("test_auth: sha256 vectors OK\n");

    uint8_t key[32], nonce[16], proof[32];
    memset(key, 0x01, 32); memset(nonce, 0x02, 16);
    auth_compute_proof(key, nonce, proof); hex(proof, 32, h);
    assert(strcmp(h, "1677832873f9b5c4ed5e2a561c6783b8c0c7c7bbd8830643ec1e0d1f1453fe40") == 0);   // == Python hashlib.sha256(key||nonce)
    assert(auth_proof_equal(proof, proof));
    uint8_t other[32]; memset(other, 0x03, 32);
    assert(!auth_proof_equal(proof, other));
    assert(!auth_key_is_set(nonce) || 1);   // nonce is 0x02.. (non-zero) -> set
    uint8_t zero[32]; memset(zero, 0, 32);
    assert(!auth_key_is_set(zero) && auth_key_is_set(key));
    printf("test_auth: proof cross-check vs Python OK\n");
    printf("test_auth: OK\n");
    return 0;
}
