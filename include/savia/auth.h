// BLE auth (app-level shared secret, challenge-response). The app stores
// auth_key = SHA256(password) on the device; on connect the device issues a
// nonce and the app proves knowledge with proof = SHA256(auth_key || nonce).
// SDK-free, host-testable.
#ifndef SAVIA_AUTH_H
#define SAVIA_AUTH_H

#include <stdint.h>
#include <stdbool.h>

#define SAVIA_AUTH_KEY_LEN   32   // SHA256(password)
#define SAVIA_AUTH_NONCE_LEN 16
#define SAVIA_AUTH_PROOF_LEN 32   // SHA256 digest

void auth_compute_proof(const uint8_t key[SAVIA_AUTH_KEY_LEN],
                        const uint8_t nonce[SAVIA_AUTH_NONCE_LEN],
                        uint8_t out[SAVIA_AUTH_PROOF_LEN]);

bool auth_key_is_set(const uint8_t key[SAVIA_AUTH_KEY_LEN]);                 // provisioned?
bool auth_proof_equal(const uint8_t a[SAVIA_AUTH_PROOF_LEN],
                      const uint8_t b[SAVIA_AUTH_PROOF_LEN]);                // constant-time

#endif // SAVIA_AUTH_H
