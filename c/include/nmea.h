/* NMEA 0183 sentence parsing (GGA, RMC). C99, no dynamic allocation. */
#ifndef NAVTRACK_NMEA_H
#define NAVTRACK_NMEA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NMEA_MAX_SENTENCE 82 /* max length per the NMEA 0183 spec, incl. "$" and CRLF */

typedef enum {
    NMEA_OK = 0,
    NMEA_ERR_NULL,
    NMEA_ERR_TOO_LONG,
    NMEA_ERR_NO_START,
    NMEA_ERR_NO_CHECKSUM,
    NMEA_ERR_BAD_CHECKSUM,
    NMEA_ERR_UNSUPPORTED,
    NMEA_ERR_FIELD
} nmea_status_t;

typedef enum { NMEA_TYPE_UNKNOWN = 0, NMEA_TYPE_GGA, NMEA_TYPE_RMC } nmea_type_t;

typedef struct {
    uint8_t hour, minute;
    double second;
} nmea_time_t;

typedef struct {
    nmea_time_t time;
    double latitude_deg;   /* + north */
    double longitude_deg;  /* + east */
    uint8_t fix_quality;   /* 0 = invalid, 1 = GPS, 2 = DGPS ... */
    uint8_t satellites;
    double hdop;
    double altitude_m;
} nmea_gga_t;

typedef struct {
    nmea_time_t time;
    int valid;             /* status 'A' = 1, 'V' = 0 */
    double latitude_deg;
    double longitude_deg;
    double speed_knots;
    double course_deg;
    uint8_t day, month;
    uint16_t year;
} nmea_rmc_t;

typedef struct {
    nmea_type_t type;
    union {
        nmea_gga_t gga;
        nmea_rmc_t rmc;
    } data;
} nmea_sentence_t;

/* XOR of all bytes between '$' and '*'. */
uint8_t nmea_checksum(const char *body, size_t len);

/* Validates framing and checksum without parsing fields. */
nmea_status_t nmea_validate(const char *sentence);

/* Parses one sentence (with or without trailing CR/LF). */
nmea_status_t nmea_parse(const char *sentence, nmea_sentence_t *out);

const char *nmea_status_str(nmea_status_t status);

#ifdef __cplusplus
}
#endif

#endif
