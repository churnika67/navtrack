#include <cstdio>
#include <sstream>

#include "navtrack.hpp"

extern "C" {
#include "geo.h"
}

namespace navtrack {

TrackSummary Track::summarize() const {
    TrackSummary s;
    s.points = fixes_.size();
    if (fixes_.size() < 2) return s;
    for (std::size_t i = 1; i < fixes_.size(); ++i) {
        const Fix &a = fixes_[i - 1];
        const Fix &b = fixes_[i];
        double d = geo_distance_m(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg);
        double dt = b.time_s - a.time_s;
        s.distance_m += d;
        if (dt > 0 && d / dt > s.max_speed_mps) s.max_speed_mps = d / dt;
    }
    s.duration_s = fixes_.back().time_s - fixes_.front().time_s;
    if (s.duration_s > 0) s.avg_speed_mps = s.distance_m / s.duration_s;
    return s;
}

std::string Track::to_gpx(const std::string &name) const {
    std::ostringstream out;
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<gpx version=\"1.1\" creator=\"navtrack\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
        << "  <trk><name>" << name << "</name><trkseg>\n";
    char buf[160];
    for (const Fix &f : fixes_) {
        std::snprintf(buf, sizeof buf, "    <trkpt lat=\"%.7f\" lon=\"%.7f\"><ele>%.1f</ele></trkpt>\n", f.lat_deg,
                      f.lon_deg, f.altitude_m);
        out << buf;
    }
    out << "  </trkseg></trk>\n</gpx>\n";
    return out.str();
}

}  // namespace navtrack
