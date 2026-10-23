#include "nmea.h"

#include <stdlib.h>
#include <string.h>

#define MAX_FIELDS 20

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

uint8_t nmea_checksum(const char *body, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum ^= (uint8_t)body[i];
    return sum;
}

/* Finds the '*' and checks the two hex digits after it. */
static nmea_status_t check_frame(const char *s, size_t *body_len) {
    if (s == NULL) return NMEA_ERR_NULL;
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r')) --len;
    if (len + 2 > NMEA_MAX_SENTENCE) return NMEA_ERR_TOO_LONG;
    if (len == 0 || s[0] != '$') return NMEA_ERR_NO_START;

    const char *star = memchr(s, '*', len);
    if (star == NULL || (size_t)(star - s) + 3 != len) return NMEA_ERR_NO_CHECKSUM;
    int hi = hex_value(star[1]);
    int lo = hex_value(star[2]);
    if (hi < 0 || lo < 0) return NMEA_ERR_NO_CHECKSUM;

    size_t blen = (size_t)(star - s) - 1;
    if (nmea_checksum(s + 1, blen) != (uint8_t)((hi << 4) | lo)) return NMEA_ERR_BAD_CHECKSUM;
    *body_len = blen;
    return NMEA_OK;
}

nmea_status_t nmea_validate(const char *sentence) {
    size_t body_len;
    return check_frame(sentence, &body_len);
}

/* Splits buf in place on ','. Empty fields become "". */
static size_t split_fields(char *buf, char **fields, size_t max_fields) {
    size_t n = 0;
    fields[n++] = buf;
    for (char *p = buf; *p != '\0' && n < max_fields; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
}

static int parse_double(const char *f, double *out) {
    if (f[0] == '\0') return 0;
    char *end;
    *out = strtod(f, &end);
    return *end == '\0';
}

static int parse_uint(const char *f, unsigned long max, unsigned long *out) {
    if (f[0] == '\0') return 0;
    char *end;
    unsigned long v = strtoul(f, &end, 10);
    if (*end != '\0' || v > max) return 0;
    *out = v;
    return 1;
}

/* hhmmss(.sss) */
static int parse_time(const char *f, nmea_time_t *t) {
    if (strlen(f) < 6) return 0;
    for (int i = 0; i < 6; ++i)
        if (f[i] < '0' || f[i] > '9') return 0;
    t->hour = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
    t->minute = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
    double sec;
    if (!parse_double(f + 4, &sec)) return 0;
    t->second = sec;
    return t->hour < 24 && t->minute < 60 && sec < 61.0;
}

/* (d)ddmm.mmmm + hemisphere -> signed decimal degrees */
static int parse_coord(const char *value, const char *hemi, int deg_digits, double *out) {
    size_t len = strlen(value);
    if (len < (size_t)deg_digits + 2 || hemi[0] == '\0' || hemi[1] != '\0') return 0;
    double degrees = 0.0;
    for (int i = 0; i < deg_digits; ++i) {
        if (value[i] < '0' || value[i] > '9') return 0;
        degrees = degrees * 10.0 + (value[i] - '0');
    }
    double minutes;
    if (!parse_double(value + deg_digits, &minutes) || minutes >= 60.0) return 0;
    double result = degrees + minutes / 60.0;
    double limit = deg_digits == 2 ? 90.0 : 180.0;
    if (result > limit) return 0;

    char h = hemi[0];
    if (deg_digits == 2 && h != 'N' && h != 'S') return 0;
    if (deg_digits == 3 && h != 'E' && h != 'W') return 0;
    *out = (h == 'S' || h == 'W') ? -result : result;
    return 1;
}

static nmea_status_t parse_gga(char **f, size_t n, nmea_gga_t *g) {
    if (n < 15) return NMEA_ERR_FIELD;
    unsigned long q, sats;
    memset(g, 0, sizeof(*g));
    if (!parse_time(f[1], &g->time)) return NMEA_ERR_FIELD;
    if (!parse_uint(f[6], 8, &q)) return NMEA_ERR_FIELD;
    g->fix_quality = (uint8_t)q;
    if (q == 0) return NMEA_OK; /* no fix: position fields may be empty */
    if (!parse_coord(f[2], f[3], 2, &g->latitude_deg)) return NMEA_ERR_FIELD;
    if (!parse_coord(f[4], f[5], 3, &g->longitude_deg)) return NMEA_ERR_FIELD;
    if (!parse_uint(f[7], 99, &sats)) return NMEA_ERR_FIELD;
    g->satellites = (uint8_t)sats;
    if (!parse_double(f[8], &g->hdop)) return NMEA_ERR_FIELD;
    if (!parse_double(f[9], &g->altitude_m)) return NMEA_ERR_FIELD;
    return NMEA_OK;
}

static nmea_status_t parse_rmc(char **f, size_t n, nmea_rmc_t *r) {
    if (n < 12) return NMEA_ERR_FIELD;
    memset(r, 0, sizeof(*r));
    if (!parse_time(f[1], &r->time)) return NMEA_ERR_FIELD;
    if (f[2][0] != 'A' && f[2][0] != 'V') return NMEA_ERR_FIELD;
    r->valid = f[2][0] == 'A';
    if (!r->valid) return NMEA_OK;
    if (!parse_coord(f[3], f[4], 2, &r->latitude_deg)) return NMEA_ERR_FIELD;
    if (!parse_coord(f[5], f[6], 3, &r->longitude_deg)) return NMEA_ERR_FIELD;
    if (!parse_double(f[7], &r->speed_knots)) return NMEA_ERR_FIELD;
    if (f[8][0] != '\0' && !parse_double(f[8], &r->course_deg)) return NMEA_ERR_FIELD;

    const char *d = f[9];
    if (strlen(d) != 6) return NMEA_ERR_FIELD;
    for (int i = 0; i < 6; ++i)
        if (d[i] < '0' || d[i] > '9') return NMEA_ERR_FIELD;
    r->day = (uint8_t)((d[0] - '0') * 10 + (d[1] - '0'));
    r->month = (uint8_t)((d[2] - '0') * 10 + (d[3] - '0'));
    r->year = (uint16_t)(2000 + (d[4] - '0') * 10 + (d[5] - '0'));
    if (r->day < 1 || r->day > 31 || r->month < 1 || r->month > 12) return NMEA_ERR_FIELD;
    return NMEA_OK;
}

nmea_status_t nmea_parse(const char *sentence, nmea_sentence_t *out) {
    if (out == NULL) return NMEA_ERR_NULL;
    out->type = NMEA_TYPE_UNKNOWN;

    size_t body_len;
    nmea_status_t st = check_frame(sentence, &body_len);
    if (st != NMEA_OK) return st;
    if (body_len < 5) return NMEA_ERR_UNSUPPORTED;

    char buf[NMEA_MAX_SENTENCE];
    memcpy(buf, sentence + 1, body_len);
    buf[body_len] = '\0';

    char *fields[MAX_FIELDS];
    size_t n = split_fields(buf, fields, MAX_FIELDS);
    const char *id = fields[0];
    if (strlen(id) != 5) return NMEA_ERR_UNSUPPORTED;

    /* Any talker ID (GP, GN, GL, ...) is accepted. */
    if (strcmp(id + 2, "GGA") == 0) {
        out->type = NMEA_TYPE_GGA;
        return parse_gga(fields, n, &out->data.gga);
    }
    if (strcmp(id + 2, "RMC") == 0) {
        out->type = NMEA_TYPE_RMC;
        return parse_rmc(fields, n, &out->data.rmc);
    }
    return NMEA_ERR_UNSUPPORTED;
}

const char *nmea_status_str(nmea_status_t status) {
    switch (status) {
    case NMEA_OK: return "ok";
    case NMEA_ERR_NULL: return "null input";
    case NMEA_ERR_TOO_LONG: return "sentence too long";
    case NMEA_ERR_NO_START: return "missing '$'";
    case NMEA_ERR_NO_CHECKSUM: return "missing checksum";
    case NMEA_ERR_BAD_CHECKSUM: return "checksum mismatch";
    case NMEA_ERR_UNSUPPORTED: return "unsupported sentence";
    case NMEA_ERR_FIELD: return "malformed field";
    }
    return "unknown";
}
