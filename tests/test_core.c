/* Unit tests for the C core. No framework, so it also builds for bare-metal targets. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "geo.h"
#include "nmea.h"
#include "ring_buffer.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        checks++;                                                         \
        if (!(cond)) {                                                    \
            failures++;                                                   \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

static const char *GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
static const char *RMC = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";

static void test_checksum(void) {
    CHECK(nmea_validate(GGA) == NMEA_OK);
    CHECK(nmea_validate(RMC) == NMEA_OK);

    char bad[100];
    strcpy(bad, GGA);
    bad[10] = '9'; /* corrupt one digit */
    CHECK(nmea_validate(bad) == NMEA_ERR_BAD_CHECKSUM);

    CHECK(nmea_validate("GPGGA,123519*47") == NMEA_ERR_NO_START);
    CHECK(nmea_validate("$GPGGA,123519") == NMEA_ERR_NO_CHECKSUM);
    CHECK(nmea_validate("$GPGGA,123519*4") == NMEA_ERR_NO_CHECKSUM);
    CHECK(nmea_validate("$GPGGA,123519*ZZ") == NMEA_ERR_NO_CHECKSUM);
    CHECK(nmea_validate(NULL) == NMEA_ERR_NULL);
    CHECK(nmea_validate("") == NMEA_ERR_NO_START);

    char lower[100];
    strcpy(lower, RMC);
    lower[strlen(lower) - 1] = 'a'; /* lowercase hex is accepted */
    CHECK(nmea_validate(lower) == NMEA_OK);
}

static void test_too_long(void) {
    char longs[200];
    memset(longs, 'A', sizeof longs);
    longs[0] = '$';
    longs[150] = '*';
    longs[151] = '0';
    longs[152] = '0';
    longs[153] = '\0';
    CHECK(nmea_validate(longs) == NMEA_ERR_TOO_LONG);
}

static void test_parse_gga(void) {
    nmea_sentence_t s;
    CHECK(nmea_parse(GGA, &s) == NMEA_OK);
    CHECK(s.type == NMEA_TYPE_GGA);
    CHECK(s.data.gga.time.hour == 12 && s.data.gga.time.minute == 35);
    CHECK_NEAR(s.data.gga.time.second, 19.0, 1e-9);
    CHECK_NEAR(s.data.gga.latitude_deg, 48.0 + 7.038 / 60.0, 1e-9);
    CHECK_NEAR(s.data.gga.longitude_deg, 11.0 + 31.0 / 60.0, 1e-9);
    CHECK(s.data.gga.fix_quality == 1);
    CHECK(s.data.gga.satellites == 8);
    CHECK_NEAR(s.data.gga.hdop, 0.9, 1e-9);
    CHECK_NEAR(s.data.gga.altitude_m, 545.4, 1e-9);

    /* With CRLF, as it arrives on the wire. */
    char wire[100];
    snprintf(wire, sizeof wire, "%s\r\n", GGA);
    CHECK(nmea_parse(wire, &s) == NMEA_OK);
}

static void test_parse_rmc(void) {
    nmea_sentence_t s;
    CHECK(nmea_parse(RMC, &s) == NMEA_OK);
    CHECK(s.type == NMEA_TYPE_RMC);
    CHECK(s.data.rmc.valid == 1);
    CHECK_NEAR(s.data.rmc.speed_knots, 22.4, 1e-9);
    CHECK_NEAR(s.data.rmc.course_deg, 84.4, 1e-9);
    CHECK(s.data.rmc.day == 23 && s.data.rmc.month == 3 && s.data.rmc.year == 2094);
}

/* Builds a sentence with a correct checksum from its body. */
static void make_sentence(const char *body, char *out, size_t size) {
    snprintf(out, size, "$%s*%02X", body, nmea_checksum(body, strlen(body)));
}

static void test_hemispheres_and_talkers(void) {
    char buf[100];
    nmea_sentence_t s;
    make_sentence("GNGGA,000001.00,3347.9440,S,15112.7360,W,2,12,0.7,10.0,M,0.0,M,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_OK);
    CHECK(s.data.gga.latitude_deg < 0);
    CHECK(s.data.gga.longitude_deg < 0);
    CHECK_NEAR(s.data.gga.latitude_deg, -(33.0 + 47.944 / 60.0), 1e-9);
    CHECK(s.data.gga.fix_quality == 2);
}

static void test_no_fix_and_bad_fields(void) {
    char buf[100];
    nmea_sentence_t s;

    make_sentence("GPGGA,120000,,,,,0,00,,,M,,M,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_OK);
    CHECK(s.data.gga.fix_quality == 0);

    make_sentence("GPRMC,120000,V,,,,,,,231026,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_OK);
    CHECK(s.data.rmc.valid == 0);

    make_sentence("GPGGA,126000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_ERR_FIELD); /* minute 60 */

    make_sentence("GPGGA,123519,4807.038,X,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_ERR_FIELD); /* bad hemisphere */

    make_sentence("GPGGA,123519,4867.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_ERR_FIELD); /* 67 minutes */

    make_sentence("GPGGA,123519,4807.038,N", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_ERR_FIELD); /* truncated */

    make_sentence("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,231394,,", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_ERR_FIELD); /* month 13 */

    make_sentence("GPGSV,3,1,11,03,03,111,00", buf, sizeof buf);
    CHECK(nmea_parse(buf, &s) == NMEA_ERR_UNSUPPORTED);
}

static void test_ring_buffer(void) {
    uint8_t storage[8];
    ring_buffer_t rb;
    rb_init(&rb, storage, sizeof storage);
    uint8_t b;
    CHECK(rb_pop(&rb, &b) == 0);

    CHECK(rb_write(&rb, (const uint8_t *)"abcdefghij", 10) == 8);
    CHECK(rb_is_full(&rb));
    CHECK(rb.dropped == 2);
    CHECK(rb_pop(&rb, &b) == 1 && b == 'a');

    /* Wrap-around keeps byte order. */
    CHECK(rb_push(&rb, 'X') == 1);
    char out[16];
    int i = 0;
    while (rb_pop(&rb, &b)) out[i++] = (char)b;
    out[i] = '\0';
    CHECK(strcmp(out, "bcdefghX") == 0);
}

static void test_ring_buffer_lines(void) {
    uint8_t storage[32];
    ring_buffer_t rb;
    rb_init(&rb, storage, sizeof storage);
    char line[16];

    rb_write(&rb, (const uint8_t *)"abc", 3);
    CHECK(rb_read_line(&rb, line, sizeof line) == RB_NO_LINE);
    rb_write(&rb, (const uint8_t *)"\r\n\r\nxyz\n", 8);
    CHECK(rb_read_line(&rb, line, sizeof line) == 3 && strcmp(line, "abc") == 0);
    CHECK(rb_read_line(&rb, line, sizeof line) == 0); /* empty line */
    CHECK(rb_read_line(&rb, line, sizeof line) == 3 && strcmp(line, "xyz") == 0);
    CHECK(rb_count(&rb) == 0);

    /* A line longer than the output buffer is consumed and reported. */
    rb_write(&rb, (const uint8_t *)"0123456789ABCDEFGH\nok\n", 22);
    CHECK(rb_read_line(&rb, line, sizeof line) == RB_LINE_TOO_LONG);
    CHECK(rb_read_line(&rb, line, sizeof line) == 2 && strcmp(line, "ok") == 0);

    /* A full buffer with no newline is dropped so the stream can resync. */
    uint8_t junk[32];
    memset(junk, 'z', sizeof junk);
    rb_write(&rb, junk, sizeof junk);
    CHECK(rb_read_line(&rb, line, sizeof line) == RB_NO_LINE);
    CHECK(rb_count(&rb) == 0);
}

static void test_geo(void) {
    /* One degree of latitude is about 111.2 km. */
    CHECK_NEAR(geo_distance_m(0, 0, 1, 0), 111195.0, 5.0);
    CHECK_NEAR(geo_distance_m(35.0, -78.0, 35.0, -78.0), 0.0, 1e-9);
    /* London to Paris, about 343.5 km. */
    CHECK_NEAR(geo_distance_m(51.5074, -0.1278, 48.8566, 2.3522) / 1000.0, 343.5, 1.0);

    CHECK_NEAR(geo_bearing_deg(0, 0, 1, 0), 0.0, 1e-6);
    CHECK_NEAR(geo_bearing_deg(0, 0, 0, 1), 90.0, 1e-6);
    CHECK_NEAR(geo_bearing_deg(0, 0, -1, 0), 180.0, 1e-6);
    CHECK_NEAR(geo_bearing_deg(0, 0, 0, -1), 270.0, 1e-6);

    double lat, lon;
    geo_destination(35.79, -78.78, 45.0, 1000.0, &lat, &lon);
    CHECK_NEAR(geo_distance_m(35.79, -78.78, lat, lon), 1000.0, 1e-3);
    CHECK_NEAR(geo_bearing_deg(35.79, -78.78, lat, lon), 45.0, 1e-3);

    /* Crossing the antimeridian stays in [-180, 180). */
    geo_destination(0.0, 179.9, 90.0, 50000.0, &lat, &lon);
    CHECK(lon < -179.0 && lon >= -180.0);
}

int main(void) {
    test_checksum();
    test_too_long();
    test_parse_gga();
    test_parse_rmc();
    test_hemispheres_and_talkers();
    test_no_fix_and_bad_fields();
    test_ring_buffer();
    test_ring_buffer_lines();
    test_geo();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
