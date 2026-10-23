// C++17 layer on top of the C core: stream decoding, filtering, track stats, simulation.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

extern "C" {
#include "nmea.h"
#include "ring_buffer.h"
}

namespace navtrack {

constexpr double kKnotsToMps = 0.514444;

struct Fix {
    double time_s = 0.0;  // seconds since midnight UTC
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double speed_mps = 0.0;
    double course_deg = 0.0;
    double hdop = 0.0;
    double altitude_m = 0.0;
    int satellites = 0;
};

struct DecoderStats {
    std::size_t sentences = 0;
    std::size_t parsed = 0;
    std::size_t checksum_errors = 0;
    std::size_t field_errors = 0;
    std::size_t framing_errors = 0;
    std::size_t unsupported = 0;
    std::size_t oversized_lines = 0;
    std::size_t no_fix = 0;
};

// Feeds raw serial bytes through a fixed-size ring buffer (like a UART RX queue)
// and pairs GGA + RMC sentences that share a timestamp into one Fix.
class NmeaStreamDecoder {
public:
    static constexpr std::size_t kBufferSize = 512;

    NmeaStreamDecoder();
    NmeaStreamDecoder(const NmeaStreamDecoder &) = delete;
    NmeaStreamDecoder &operator=(const NmeaStreamDecoder &) = delete;

    // Returns the number of bytes accepted; the rest were dropped (buffer full).
    std::size_t feed(const std::uint8_t *data, std::size_t len);
    std::size_t feed(const std::string &chunk);

    // Moves completed fixes out of the decoder.
    std::vector<Fix> take_fixes();
    // Emits a pending half-built fix (e.g. a trailing GGA with no RMC).
    void flush();

    const DecoderStats &stats() const { return stats_; }
    std::size_t dropped_bytes() const { return rb_.dropped; }

private:
    void drain_lines();
    void handle(const nmea_sentence_t &s);
    void commit_pending();

    std::array<std::uint8_t, kBufferSize> storage_{};
    ring_buffer_t rb_{};
    DecoderStats stats_{};
    std::optional<Fix> pending_;
    bool pending_has_gga_ = false;
    bool pending_has_rmc_ = false;
    std::vector<Fix> fixes_;
};

// Constant-velocity Kalman filter, run independently on the east and north axes
// of a local tangent plane anchored at the first fix.
class KalmanSmoother {
public:
    explicit KalmanSmoother(double accel_noise_mps2 = 1.0, double uere_m = 5.0);
    Fix update(const Fix &raw);
    void reset();

private:
    struct Axis {
        double pos = 0.0, vel = 0.0;
        double p00 = 0.0, p01 = 0.0, p11 = 0.0;
        void predict(double dt, double q);
        void correct(double z, double r);
    };
    double accel_noise_;
    double uere_m_;
    bool initialised_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0, last_time_ = 0.0;
    Axis east_, north_;
};

struct TrackSummary {
    std::size_t points = 0;
    double distance_m = 0.0;
    double duration_s = 0.0;
    double max_speed_mps = 0.0;
    double avg_speed_mps = 0.0;
};

class Track {
public:
    void add(const Fix &fix) { fixes_.push_back(fix); }
    const std::vector<Fix> &fixes() const { return fixes_; }
    bool empty() const { return fixes_.empty(); }
    TrackSummary summarize() const;
    std::string to_gpx(const std::string &name) const;

private:
    std::vector<Fix> fixes_;
};

struct Waypoint {
    double lat_deg;
    double lon_deg;
};

struct SimConfig {
    std::vector<Waypoint> route;
    double speed_mps = 12.0;
    double noise_m = 6.0;           // 1-sigma horizontal position noise
    double corrupt_probability = 0.0;
    double start_time_s = 12 * 3600.0;
    unsigned seed = 42;
};

struct SimResult {
    std::string nmea;          // GGA + RMC per second
    std::vector<Fix> truth;    // noise-free positions
};

// Drives a receiver along a route at 1 Hz and writes noisy NMEA, optionally
// corrupting bytes to exercise checksum handling.
SimResult simulate(const SimConfig &cfg, double seconds);

std::string format_gga(const Fix &fix);
std::string format_rmc(const Fix &fix);

}  // namespace navtrack
