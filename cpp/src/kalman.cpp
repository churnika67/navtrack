#include <cmath>

#include "navtrack.hpp"

extern "C" {
#include "geo.h"
}

namespace navtrack {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

KalmanSmoother::KalmanSmoother(double accel_noise_mps2, double uere_m)
    : accel_noise_(accel_noise_mps2), uere_m_(uere_m) {}

void KalmanSmoother::reset() { initialised_ = false; }

void KalmanSmoother::Axis::predict(double dt, double q) {
    // x' = F x, P' = F P F^T + Q for F = [1 dt; 0 1] and white-noise acceleration.
    pos += vel * dt;
    double n00 = p00 + dt * (2 * p01 + dt * p11);
    double n01 = p01 + dt * p11;
    double dt2 = dt * dt;
    p00 = n00 + q * dt2 * dt2 / 4.0;
    p01 = n01 + q * dt2 * dt / 2.0;
    p11 = p11 + q * dt2;
}

void KalmanSmoother::Axis::correct(double z, double r) {
    double s = p00 + r;
    double k0 = p00 / s;
    double k1 = p01 / s;
    double y = z - pos;
    pos += k0 * y;
    vel += k1 * y;
    double n00 = (1 - k0) * p00;
    double n01 = (1 - k0) * p01;
    double n11 = p11 - k1 * p01;
    p00 = n00;
    p01 = n01;
    p11 = n11;
}

Fix KalmanSmoother::update(const Fix &raw) {
    double sigma = (raw.hdop > 0 ? raw.hdop : 1.0) * uere_m_;
    double r = sigma * sigma;
    double scale = GEO_EARTH_RADIUS_M * kDegToRad;

    if (!initialised_ || raw.time_s <= last_time_) {
        origin_lat_ = raw.lat_deg;
        origin_lon_ = raw.lon_deg;
        east_ = Axis{0.0, 0.0, r, 0.0, 100.0};
        north_ = Axis{0.0, 0.0, r, 0.0, 100.0};
        last_time_ = raw.time_s;
        initialised_ = true;
        return raw;
    }

    double dt = raw.time_s - last_time_;
    last_time_ = raw.time_s;
    double q = accel_noise_ * accel_noise_;
    double e = (raw.lon_deg - origin_lon_) * scale * std::cos(origin_lat_ * kDegToRad);
    double n = (raw.lat_deg - origin_lat_) * scale;

    east_.predict(dt, q);
    north_.predict(dt, q);
    east_.correct(e, r);
    north_.correct(n, r);

    Fix out = raw;
    out.lat_deg = origin_lat_ + north_.pos / scale;
    out.lon_deg = origin_lon_ + east_.pos / (scale * std::cos(origin_lat_ * kDegToRad));
    out.speed_mps = std::hypot(east_.vel, north_.vel);
    return out;
}

}  // namespace navtrack
