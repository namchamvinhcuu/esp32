/* Host-side unit test for scale_parse.c (no ESP-IDF needed).
 * Build & run:
 *   gcc -I../../components/scale_serial test_scale_parse.c \
 *       ../../components/scale_serial/scale_parse.c -o test_scale_parse
 *   ./test_scale_parse
 * Add every REAL line captured from the customer's scale (capture_scale.py)
 * as a new case here before trusting the parser with it. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "scale_parse.h"

static int g_fail;

static void check(const char *line, int exp_ok, int exp_stable, float exp_val)
{
    scale_reading_t r;
    int ok = scale_parse_line(line, &r);
    int pass = (ok == exp_ok) &&
               (!exp_ok || (r.stable == exp_stable &&
                            fabsf(r.value - exp_val) < 0.001f));
    printf("%-32s -> ok=%d stable=%d value=%.3f  [%s]\n",
           line, ok, r.stable, r.value, pass ? "PASS" : "FAIL");
    if (!pass) {
        g_fail++;
    }
}

int main(void)
{
    /* CAS / A&D continuous output */
    check("ST,GS,+  123.45 g",   1, 1, 123.45f);
    check("US,GS,+  123.47 g",   1, 0, 123.47f);
    check("ST,NT,-   0.05 kg",   1, 1, -0.05f);
    check("ST,GS,+00123.4 g",    1, 1, 123.4f);
    check("US,NT,+   0.00 g",    1, 0, 0.0f);
    check("OL,GS,+9999.99 g",    0, 0, 0.0f);

    /* generic fallbacks */
    check("  456.78 g",          1, 1, 456.78f);
    check("W:+12.3kg",           1, 1, 12.3f);
    check("-0.5",                1, 1, -0.5f);

    /* junk must not parse */
    check("",                    0, 0, 0.0f);
    check("READY",               0, 0, 0.0f);
    check(",,,",                 0, 0, 0.0f);

    if (g_fail) {
        printf("\n%d FAILURE(S)\n", g_fail);
        return 1;
    }
    printf("\nALL PASS\n");
    return 0;
}
