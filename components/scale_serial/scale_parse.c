#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "scale_parse.h"

/* Extract the first signed decimal number from s, tolerating spaces between
 * the sign and the digits ("+  123.45", common in CAS/A&D right-aligned
 * fields). Returns true and writes *val on success. */
static bool extract_number(const char *s, float *val)
{
    /* compact copy: drop spaces so "+  123.45" -> "+123.45" */
    char buf[64];
    size_t n = 0;
    for (const char *p = s; *p != '\0' && n < sizeof(buf) - 1; p++) {
        if (*p != ' ' && *p != '\t') {
            buf[n++] = *p;
        }
    }
    buf[n] = '\0';

    for (size_t i = 0; i < n; i++) {
        char c = buf[i];
        bool starts_num = isdigit((unsigned char)c) ||
                          ((c == '+' || c == '-' || c == '.') &&
                           i + 1 < n && isdigit((unsigned char)buf[i + 1]));
        if (starts_num) {
            char *end = NULL;
            float v = strtof(&buf[i], &end);
            if (end != &buf[i]) {
                *val = v;
                return true;
            }
        }
    }
    return false;
}

bool scale_parse_line(const char *line, scale_reading_t *out)
{
    out->ok     = false;
    out->stable = true;
    out->value  = 0.0f;

    if (line == NULL) {
        return false;
    }
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line == '\0') {
        return false;
    }

    /* CAS / A&D header: two letters + comma, e.g. "ST,GS,..." */
    if (isalpha((unsigned char)line[0]) && isalpha((unsigned char)line[1]) &&
        line[2] == ',') {
        if (line[0] == 'O' && line[1] == 'L') {
            return false; /* overload: no usable value */
        }
        out->stable = !(line[0] == 'U' && line[1] == 'S');
        /* skip header tokens ("ST,GS,") to avoid parsing digits in them */
        const char *p = line;
        int commas = 0;
        while (*p != '\0' && commas < 2) {
            if (*p == ',') {
                commas++;
            }
            p++;
        }
        if (extract_number(commas == 2 ? p : line, &out->value)) {
            out->ok = true;
        }
        return out->ok;
    }

    /* generic fallback: any line carrying a number ("  123.45 g", "W:+12.3") */
    if (extract_number(line, &out->value)) {
        out->ok = true;
    }
    return out->ok;
}
