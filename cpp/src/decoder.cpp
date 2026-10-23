#include "navtrack.hpp"

namespace navtrack {

namespace {
double seconds_of_day(const nmea_time_t &t) { return t.hour * 3600.0 + t.minute * 60.0 + t.second; }
bool same_time(double a, double b) { return a - b < 1e-3 && b - a < 1e-3; }
}  // namespace

NmeaStreamDecoder::NmeaStreamDecoder() { rb_init(&rb_, storage_.data(), storage_.size()); }

std::size_t NmeaStreamDecoder::feed(const std::uint8_t *data, std::size_t len) {
    std::size_t accepted = 0;
    // Push in pieces and drain between them, the way an RX interrupt and the
    // main loop share a UART queue.
    while (len > 0) {
        std::size_t room = rb_.capacity - rb_count(&rb_);
        std::size_t n = len < room ? len : room;
        if (n == 0) {
            drain_lines();
            room = rb_.capacity - rb_count(&rb_);
            if (room == 0) {  // nothing could be drained: count the overflow
                accepted += rb_write(&rb_, data, len);
                break;
            }
            continue;
        }
        accepted += rb_write(&rb_, data, n);
        data += n;
        len -= n;
        drain_lines();
    }
    return accepted;
}

std::size_t NmeaStreamDecoder::feed(const std::string &chunk) {
    return feed(reinterpret_cast<const std::uint8_t *>(chunk.data()), chunk.size());
}

void NmeaStreamDecoder::drain_lines() {
    char line[NMEA_MAX_SENTENCE + 1];
    for (;;) {
        int n = rb_read_line(&rb_, line, sizeof line);
        if (n == RB_NO_LINE) return;
        if (n == 0) continue;  // blank line between sentences
        stats_.sentences++;
        if (n == RB_LINE_TOO_LONG) {
            stats_.oversized_lines++;
            continue;
        }
        nmea_sentence_t s;
        nmea_status_t st = nmea_parse(line, &s);
        switch (st) {
        case NMEA_OK:
            stats_.parsed++;
            handle(s);
            break;
        case NMEA_ERR_BAD_CHECKSUM: stats_.checksum_errors++; break;
        case NMEA_ERR_FIELD: stats_.field_errors++; break;
        case NMEA_ERR_UNSUPPORTED: stats_.unsupported++; break;
        default: stats_.framing_errors++; break;
        }
    }
}

void NmeaStreamDecoder::handle(const nmea_sentence_t &s) {
    if (s.type == NMEA_TYPE_GGA && s.data.gga.fix_quality == 0) {
        stats_.no_fix++;
        return;
    }
    if (s.type == NMEA_TYPE_RMC && !s.data.rmc.valid) {
        stats_.no_fix++;
        return;
    }

    double t = s.type == NMEA_TYPE_GGA ? seconds_of_day(s.data.gga.time) : seconds_of_day(s.data.rmc.time);
    if (pending_ && !same_time(pending_->time_s, t)) commit_pending();
    if (!pending_) {
        pending_ = Fix{};
        pending_->time_s = t;
    }

    if (s.type == NMEA_TYPE_GGA) {
        const nmea_gga_t &g = s.data.gga;
        pending_->lat_deg = g.latitude_deg;
        pending_->lon_deg = g.longitude_deg;
        pending_->hdop = g.hdop;
        pending_->altitude_m = g.altitude_m;
        pending_->satellites = g.satellites;
        pending_has_gga_ = true;
    } else {
        const nmea_rmc_t &r = s.data.rmc;
        if (!pending_has_gga_) {
            pending_->lat_deg = r.latitude_deg;
            pending_->lon_deg = r.longitude_deg;
        }
        pending_->speed_mps = r.speed_knots * kKnotsToMps;
        pending_->course_deg = r.course_deg;
        pending_has_rmc_ = true;
    }
    if (pending_has_gga_ && pending_has_rmc_) commit_pending();
}

void NmeaStreamDecoder::commit_pending() {
    if (pending_) fixes_.push_back(*pending_);
    pending_.reset();
    pending_has_gga_ = pending_has_rmc_ = false;
}

void NmeaStreamDecoder::flush() {
    drain_lines();
    commit_pending();
}

std::vector<Fix> NmeaStreamDecoder::take_fixes() {
    std::vector<Fix> out;
    out.swap(fixes_);
    return out;
}

}  // namespace navtrack
