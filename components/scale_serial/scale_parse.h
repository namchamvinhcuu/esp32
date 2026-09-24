/* scale_parse: PURE C line parser for serial scale output. No IDF includes —
 * host-testable with gcc (see firmware/fms-node/tests/host/). */
#ifndef FMS_SCALE_PARSE_H
#define FMS_SCALE_PARSE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  ok;     /* a numeric weight was extracted */
    bool  stable; /* ST -> true, US -> false, unknown format -> true */
    float value;
} scale_reading_t;

/* Parse one line (no CR/LF). Supported:
 *   CAS / A&D continuous: "ST,GS,+  123.45 g" / "US,NT,-  0.05 kg"
 *   (OL = overload -> ok=false)
 *   generic fallback: first signed decimal number found in the line
 * Returns out->ok. */
bool scale_parse_line(const char *line, scale_reading_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FMS_SCALE_PARSE_H */
