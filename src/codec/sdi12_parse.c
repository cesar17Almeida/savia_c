#include "savia/sdi12.h"

// Pure SDI-12 reply parsing (no SDK). Formats verified against the real
// AquaCheck probe -- see tools/sdi12_bringup/AQUACHECK_RESPONSES.md.

static bool is_digit(char c) { return c >= '0' && c <= '9'; }

bool sdi12_parse_measure_hdr(const char *resp, int *delay_s, int *nvals) {
    // "atttn": address + 3-digit seconds + 1-digit count (e.g. "00024" = 2 s, 4).
    if (!resp) return false;
    for (int i = 0; i < 5; i++) if (!resp[i]) return false;   // too short
    for (int i = 1; i <= 4; i++) if (!is_digit(resp[i])) return false;
    *delay_s = (resp[1] - '0') * 100 + (resp[2] - '0') * 10 + (resp[3] - '0');
    *nvals = resp[4] - '0';
    return true;
}

int sdi12_parse_values(const char *resp, float *out, int got, int max) {
    if (!resp || !resp[0]) return got;
    const char *p = resp + 1;                       // skip the address char
    while (*p && got < max) {
        while (*p && *p != '+' && *p != '-') p++;   // seek next sign
        if (!*p) break;
        float sign = (*p == '-') ? -1.0f : 1.0f;
        p++;
        float v = 0.0f, frac = 0.1f;
        bool any = false, in_frac = false;
        while (*p) {
            if (is_digit(*p)) {
                if (in_frac) { v += (float)(*p - '0') * frac; frac *= 0.1f; }
                else         { v = v * 10.0f + (float)(*p - '0'); }
                any = true;
            } else if (*p == '.' && !in_frac) {
                in_frac = true;
            } else {
                break;                              // next sign or garbage
            }
            p++;
        }
        if (any) out[got++] = sign * v;
    }
    return got;
}
