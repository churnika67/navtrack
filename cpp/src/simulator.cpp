#include <cmath>
#include <cstdio>

#include "navtrack.hpp"

extern "C" {
#include "geo.h"
}

namespace navtrack {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

void format_time(double t, char *out, std::size_t size) {
    int whole = static_cast<int>(t);
    int h = (whole / 3600) % 24, m = (whole / 60) % 60;
    double s = t - (whole - whole % 60);
    std::snprintf(out, size, "%02d%02d%05.2f", h, m, s);
}

void format_coord(double value, bool is_lat, char *out, std::size_t size) {
    double a = std::fabs(value);
    int degrees = static_cast<int>(a);
    double minutes = (a - degrees) * 60.0;
    if (minutes >= 59.99995) {  // avoid printing "60.0000" after rounding
        degrees += 1;
        minutes = 0.0;
    }
    char hemi = is_lat ? (value < 0 ? 'S' : 'N') : (value < 0 ? 'W' : 'E');
    if (is_lat)
        std::snprintf(out, size, "%02d%07.4f,%c", degrees, minutes, hemi);
    else
        std::snprintf(out, size, "%03d%07.4f,%c", degrees, minutes, hemi);
}

std::string wrap(const std::string &body) {
    char tail[8];
    std::snprintf(tail, sizeof tail, "*%02X\r\n", nmea_checksum(body.data(), body.size()));
    return "$" + body + tail;
}

}  // namespace

std::string format_gga(const Fix &f) {
    char t[16], lat[24], lon[24], body[96];
    format_time(f.time_s, t, sizeof t);
    format_coord(f.lat_deg, true, lat, sizeof lat);
    format_coord(f.lon_deg, false, lon, sizeof lon);
    std::snprintf(body, sizeof body, "GPGGA,%s,%s,%s,1,%02d,%.1f,%.1f,M,-34.0,M,,", t, lat, lon, f.satellites,
                  f.hdop, f.altitude_m);
    return wrap(body);
}

std::string format_rmc(const Fix &f) {
    char t[16], lat[24], lon[24], body[96];
    format_time(f.time_s, t, sizeof t);
    format_coord(f.lat_deg, true, lat, sizeof lat);
    format_coord(f.lon_deg, false, lon, sizeof lon);
    std::snprintf(body, sizeof body, "GPRMC,%s,A,%s,%s,%.1f,%.1f,231026,,", t, lat, lon,
                  f.speed_mps / kKnotsToMps, f.course_deg);
    return wrap(body);
}

SimResult simulate(const SimConfig &cfg, double seconds) {
    SimResult result;
    if (cfg.route.size() < 2) return result;

    std::mt19937 rng(cfg.seed);
    std::normal_distribution<double> noise(0.0, cfg.noise_m);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    std::size_t leg = 0;
    double lat = cfg.route[0].lat_deg, lon = cfg.route[0].lon_deg;

    for (int i = 0; i <= static_cast<int>(seconds); ++i) {
        if (i > 0) {
            // Advance one second along the route, carrying over into the next leg.
            double remaining = cfg.speed_mps;
            while (remaining > 0 && leg + 1 < cfg.route.size()) {
                const Waypoint &next = cfg.route[leg + 1];
                double to_next = geo_distance_m(lat, lon, next.lat_deg, next.lon_deg);
                if (to_next <= remaining) {
                    lat = next.lat_deg;
                    lon = next.lon_deg;
                    remaining -= to_next;
                    ++leg;
                } else {
                    double b = geo_bearing_deg(lat, lon, next.lat_deg, next.lon_deg);
                    geo_destination(lat, lon, b, remaining, &lat, &lon);
                    remaining = 0;
                }
            }
        }

        Fix truth;
        truth.time_s = cfg.start_time_s + i;
        truth.lat_deg = lat;
        truth.lon_deg = lon;
        truth.speed_mps = leg + 1 < cfg.route.size() ? cfg.speed_mps : 0.0;
        truth.course_deg = leg + 1 < cfg.route.size()
                               ? geo_bearing_deg(lat, lon, cfg.route[leg + 1].lat_deg, cfg.route[leg + 1].lon_deg)
                               : 0.0;
        truth.hdop = cfg.noise_m / 5.0;
        truth.altitude_m = 100.0;
        truth.satellites = 9;
        result.truth.push_back(truth);

        Fix measured = truth;
        double de = noise(rng), dn = noise(rng);
        measured.lat_deg += dn / (GEO_EARTH_RADIUS_M * kDegToRad);
        measured.lon_deg += de / (GEO_EARTH_RADIUS_M * kDegToRad * std::cos(lat * kDegToRad));

        for (std::string sentence : {format_gga(measured), format_rmc(measured)}) {
            if (cfg.corrupt_probability > 0 && unit(rng) < cfg.corrupt_probability) {
                // Flip one payload character to mimic a bit error on the serial line.
                std::size_t star = sentence.find('*');
                std::size_t pos = 1 + static_cast<std::size_t>(unit(rng) * static_cast<double>(star - 1));
                sentence[pos] = sentence[pos] == '7' ? '8' : '7';
            }
            result.nmea += sentence;
        }
    }
    return result;
}

}  // namespace navtrack
