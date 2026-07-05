// Host test for the pure SDI-12 reply parser. The vectors are REAL probe
// replies captured from the AquaCheck (tools/sdi12_bringup/AQUACHECK_RESPONSES.md).
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "savia/sdi12.h"

static int feq(float a, float b) { return fabsf(a - b) < 0.0005f; }

int main(void) {
    int delay_s, nvals;

    // --- measure header "atttn" (real replies) ---
    assert(sdi12_parse_measure_hdr("00024", &delay_s, &nvals));   // 0M!  -> 2 s, 4 vals
    assert(delay_s == 2 && nvals == 4);
    assert(sdi12_parse_measure_hdr("00004", &delay_s, &nvals));   // 0M1! -> 0 s, 4 vals
    assert(delay_s == 0 && nvals == 4);
    assert(sdi12_parse_measure_hdr("01159", &delay_s, &nvals));   // long: 115 s, 9 vals
    assert(delay_s == 115 && nvals == 9);
    assert(!sdi12_parse_measure_hdr("0002", &delay_s, &nvals));   // too short
    assert(!sdi12_parse_measure_hdr("0", &delay_s, &nvals));
    assert(!sdi12_parse_measure_hdr("0a024", &delay_s, &nvals));  // non-digit
    printf("test_sdi12_parse: measure header OK\n");

    // --- D-reply values (real replies; signs glued, no separators) ---
    float v[8];
    int n = sdi12_parse_values("0+016.9562+025.1937+002.3312", v, 0, 8);
    assert(n == 3 && feq(v[0], 16.9562f) && feq(v[1], 25.1937f) && feq(v[2], 2.3312f));
    n = sdi12_parse_values("0+002.8218", v, n, 8);                // D1! appends the 4th
    assert(n == 4 && feq(v[3], 2.8218f));
    printf("test_sdi12_parse: multi-D append OK\n");

    // Negatives and integers without decimals.
    n = sdi12_parse_values("0-2.5+30-0.125", v, 0, 8);
    assert(n == 3 && feq(v[0], -2.5f) && feq(v[1], 30.0f) && feq(v[2], -0.125f));
    // Truncation at max; empty/garbage replies yield nothing.
    n = sdi12_parse_values("0+1+2+3+4", v, 0, 2);
    assert(n == 2);
    assert(sdi12_parse_values("0", v, 0, 8) == 0);
    assert(sdi12_parse_values("", v, 0, 8) == 0);
    assert(sdi12_parse_values(NULL, v, 0, 8) == 0);
    printf("test_sdi12_parse: signs/limits/garbage OK\n");

    printf("test_sdi12_parse: OK\n");
    return 0;
}
