// Tests for the C++ layer: stream decoding, simulation round trip, filtering, track stats.
#include <cmath>
#include <cstdio>
#include <string>

#include "navtrack.hpp"

extern "C" {
#include "geo.h"
}

using namespace navtrack;

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        checks++;                                                      \
        if (!(cond)) {                                                 \
            failures++;                                                \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                              \
    } while (0)

static SimConfig cary_loop() {
    SimConfig cfg;
    cfg.route = {{35.7915, -78.7811}, {35.7960, -78.7700}, {35.7890, -78.7620}, {35.7915, -78.7811}};
    return cfg;
}

static double rms(const std::vector<Fix> &a, const std::vector<Fix> &b) {
    double s = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        double d = geo_distance_m(a[i].lat_deg, a[i].lon_deg, b[i].lat_deg, b[i].lon_deg);
        s += d * d;
    }
    return std::sqrt(s / static_cast<double>(a.size()));
}

static void test_format_round_trip() {
    Fix f;
    f.time_s = 13 * 3600 + 5 * 60 + 7.25;
    f.lat_deg = -33.8688;
    f.lon_deg = 151.2093;
    f.speed_mps = 10.0;
    f.course_deg = 270.5;
    f.hdop = 1.2;
    f.altitude_m = 58.0;
    f.satellites = 7;

    nmea_sentence_t s;
    CHECK(nmea_parse(format_gga(f).c_str(), &s) == NMEA_OK);
    CHECK(std::fabs(s.data.gga.latitude_deg - f.lat_deg) < 1e-6);
    CHECK(std::fabs(s.data.gga.longitude_deg - f.lon_deg) < 1e-6);
    CHECK(s.data.gga.satellites == 7);
    CHECK(nmea_parse(format_rmc(f).c_str(), &s) == NMEA_OK);
    CHECK(std::fabs(s.data.rmc.speed_knots * kKnotsToMps - 10.0) < 0.06);
    CHECK(std::fabs(s.data.rmc.course_deg - 270.5) < 1e-6);
}

static void test_decoder_byte_by_byte() {
    SimConfig cfg = cary_loop();
    cfg.noise_m = 0.0;
    SimResult sim = simulate(cfg, 60);

    NmeaStreamDecoder dec;
    for (char c : sim.nmea) {
        auto byte = static_cast<std::uint8_t>(c);
        dec.feed(&byte, 1);
    }
    dec.flush();
    std::vector<Fix> fixes = dec.take_fixes();
    CHECK(fixes.size() == sim.truth.size());
    CHECK(dec.stats().parsed == 2 * sim.truth.size());
    CHECK(dec.dropped_bytes() == 0);
    if (fixes.size() == sim.truth.size()) CHECK(rms(fixes, sim.truth) < 0.2);  // NMEA rounding only
}

static void test_decoder_large_chunks() {
    SimResult sim = simulate(cary_loop(), 120);
    NmeaStreamDecoder dec;
    dec.feed(sim.nmea);  // much larger than the 512-byte ring buffer
    dec.flush();
    CHECK(dec.take_fixes().size() == sim.truth.size());
    CHECK(dec.dropped_bytes() == 0);
}

static void test_decoder_rejects_corruption() {
    SimConfig cfg = cary_loop();
    cfg.corrupt_probability = 0.2;
    SimResult sim = simulate(cfg, 200);

    NmeaStreamDecoder dec;
    dec.feed(sim.nmea);
    dec.flush();
    const DecoderStats &st = dec.stats();
    CHECK(st.checksum_errors > 0);
    CHECK(st.parsed + st.checksum_errors + st.field_errors + st.framing_errors + st.unsupported == st.sentences);
    // Every fix that survives must still be near the true path.
    std::vector<Fix> fixes = dec.take_fixes();
    CHECK(!fixes.empty());
    for (const Fix &f : fixes) {
        std::size_t i = static_cast<std::size_t>(std::lround(f.time_s - cfg.start_time_s));
        CHECK(i < sim.truth.size());
        if (i < sim.truth.size())
            CHECK(geo_distance_m(f.lat_deg, f.lon_deg, sim.truth[i].lat_deg, sim.truth[i].lon_deg) < 50.0);
    }
}

static void test_garbage_resync() {
    NmeaStreamDecoder dec;
    std::string junk(2000, 'x');  // noise with no newline, larger than the buffer
    dec.feed(junk);
    Fix f;
    f.time_s = 3600;
    f.lat_deg = 35.79;
    f.lon_deg = -78.78;
    f.hdop = 1.0;
    dec.feed("\n" + format_gga(f) + format_rmc(f));
    dec.flush();
    CHECK(dec.take_fixes().size() == 1);
    CHECK(dec.dropped_bytes() > 0);
}

static void test_kalman_reduces_error() {
    SimConfig cfg = cary_loop();
    cfg.noise_m = 8.0;
    SimResult sim = simulate(cfg, 300);
    NmeaStreamDecoder dec;
    dec.feed(sim.nmea);
    dec.flush();
    std::vector<Fix> raw = dec.take_fixes();

    KalmanSmoother kf;
    std::vector<Fix> filtered;
    for (const Fix &f : raw) filtered.push_back(kf.update(f));
    double raw_err = rms(raw, sim.truth);
    double kf_err = rms(filtered, sim.truth);
    std::printf("  kalman: raw RMS %.2f m, filtered RMS %.2f m\n", raw_err, kf_err);
    CHECK(kf_err < raw_err * 0.8);

    // Filtered speed should track the 12 m/s truth.
    double mean_speed = 0;
    for (std::size_t i = 60; i < 240; ++i) mean_speed += filtered[i].speed_mps;
    mean_speed /= 180.0;
    CHECK(std::fabs(mean_speed - 12.0) < 1.5);
}

static void test_track_summary_and_gpx() {
    Track t;
    CHECK(t.summarize().distance_m == 0.0);
    Fix a, b;
    a.time_s = 0;
    a.lat_deg = 0;
    a.lon_deg = 0;
    b.time_s = 100;
    b.lat_deg = 0.01;
    b.lon_deg = 0;
    t.add(a);
    t.add(b);
    TrackSummary s = t.summarize();
    CHECK(s.points == 2);
    CHECK(std::fabs(s.distance_m - 1111.95) < 1.0);
    CHECK(std::fabs(s.avg_speed_mps - 11.12) < 0.05);

    std::string gpx = t.to_gpx("test");
    CHECK(gpx.find("<trkpt lat=\"0.0100000\"") != std::string::npos);
    CHECK(gpx.find("</gpx>") != std::string::npos);
}

int main() {
    test_format_round_trip();
    test_decoder_byte_by_byte();
    test_decoder_large_chunks();
    test_decoder_rejects_corruption();
    test_garbage_resync();
    test_kalman_reduces_error();
    test_track_summary_and_gpx();
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
