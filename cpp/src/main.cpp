// navtrack: simulate, analyze and benchmark GPS NMEA streams.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "navtrack.hpp"

extern "C" {
#include "geo.h"
}

using namespace navtrack;

namespace {

// A short loop around downtown Cary, NC.
SimConfig default_config() {
    SimConfig cfg;
    cfg.route = {{35.7915, -78.7811}, {35.7960, -78.7700}, {35.7890, -78.7620},
                 {35.7820, -78.7710}, {35.7915, -78.7811}};
    return cfg;
}

const char *arg_value(int argc, char **argv, const char *name) {
    for (int i = 0; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
    return nullptr;
}

double arg_double(int argc, char **argv, const char *name, double fallback) {
    const char *v = arg_value(argc, argv, name);
    return v ? std::atof(v) : fallback;
}

void usage() {
    std::cerr << "usage:\n"
                 "  navtrack simulate <out.nmea> [--seconds N] [--noise M] [--corrupt P] [--seed S]\n"
                 "  navtrack analyze <in.nmea> [--chunk BYTES] [--gpx out.gpx] [--raw]\n"
                 "  navtrack benchmark [--runs N] [--noise M] [--accel A]\n";
}

int cmd_simulate(int argc, char **argv) {
    if (argc < 3) return usage(), 2;
    SimConfig cfg = default_config();
    cfg.noise_m = arg_double(argc, argv, "--noise", cfg.noise_m);
    cfg.corrupt_probability = arg_double(argc, argv, "--corrupt", 0.0);
    cfg.seed = static_cast<unsigned>(arg_double(argc, argv, "--seed", cfg.seed));
    SimResult sim = simulate(cfg, arg_double(argc, argv, "--seconds", 300));
    std::ofstream(argv[2], std::ios::binary) << sim.nmea;
    std::cout << "wrote " << sim.truth.size() << " epochs (" << sim.nmea.size() << " bytes) to " << argv[2] << "\n";
    return 0;
}

int cmd_analyze(int argc, char **argv) {
    if (argc < 3) return usage(), 2;
    std::ifstream in(argv[2], std::ios::binary);
    if (!in) {
        std::cerr << "cannot open " << argv[2] << "\n";
        return 1;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    std::string data = ss.str();

    // Feed in small chunks, like bytes arriving from a UART.
    std::size_t chunk = static_cast<std::size_t>(arg_double(argc, argv, "--chunk", 64));
    if (chunk == 0) chunk = 64;
    NmeaStreamDecoder decoder;
    for (std::size_t i = 0; i < data.size(); i += chunk) decoder.feed(data.substr(i, chunk));
    decoder.flush();

    bool raw_only = false;
    for (int i = 0; i < argc; ++i) raw_only |= std::strcmp(argv[i], "--raw") == 0;

    Track track;
    KalmanSmoother kf;
    for (const Fix &f : decoder.take_fixes()) track.add(raw_only ? f : kf.update(f));

    const DecoderStats &st = decoder.stats();
    TrackSummary sum = track.summarize();
    std::printf("sentences      %zu (parsed %zu, checksum errors %zu, field errors %zu, unsupported %zu)\n",
                st.sentences, st.parsed, st.checksum_errors, st.field_errors, st.unsupported);
    std::printf("fixes          %zu (%s)\n", sum.points, raw_only ? "raw" : "Kalman filtered");
    std::printf("distance       %.1f m\n", sum.distance_m);
    std::printf("duration       %.0f s\n", sum.duration_s);
    std::printf("avg speed      %.2f m/s\n", sum.avg_speed_mps);
    std::printf("max speed      %.2f m/s\n", sum.max_speed_mps);

    if (const char *gpx = arg_value(argc, argv, "--gpx")) {
        std::ofstream(gpx) << track.to_gpx("navtrack");
        std::printf("gpx            %s\n", gpx);
    }
    return 0;
}

double rms_error(const std::vector<Fix> &a, const std::vector<Fix> &truth) {
    double sum = 0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < a.size() && i < truth.size(); ++i) {
        double d = geo_distance_m(a[i].lat_deg, a[i].lon_deg, truth[i].lat_deg, truth[i].lon_deg);
        sum += d * d;
        ++n;
    }
    return n ? std::sqrt(sum / static_cast<double>(n)) : 0.0;
}

// Compares raw vs. filtered position error and track length against ground truth.
int cmd_benchmark(int argc, char **argv) {
    int runs = static_cast<int>(arg_double(argc, argv, "--runs", 20));
    SimConfig cfg = default_config();
    cfg.noise_m = arg_double(argc, argv, "--noise", cfg.noise_m);
    double accel = arg_double(argc, argv, "--accel", 1.0);

    double raw_rms = 0, kf_rms = 0, raw_len_err = 0, kf_len_err = 0;
    for (int r = 0; r < runs; ++r) {
        cfg.seed = 1000u + static_cast<unsigned>(r);
        SimResult sim = simulate(cfg, 300);
        NmeaStreamDecoder dec;
        dec.feed(sim.nmea);
        dec.flush();
        std::vector<Fix> raw = dec.take_fixes();

        KalmanSmoother kf(accel);
        std::vector<Fix> filtered;
        Track raw_track, kf_track, truth_track;
        for (const Fix &f : raw) {
            filtered.push_back(kf.update(f));
            raw_track.add(f);
            kf_track.add(filtered.back());
        }
        for (const Fix &f : sim.truth) truth_track.add(f);

        double truth_len = truth_track.summarize().distance_m;
        raw_rms += rms_error(raw, sim.truth);
        kf_rms += rms_error(filtered, sim.truth);
        raw_len_err += std::fabs(raw_track.summarize().distance_m - truth_len) / truth_len;
        kf_len_err += std::fabs(kf_track.summarize().distance_m - truth_len) / truth_len;
    }
    std::printf("runs                 %d x 300 s, noise %.1f m (1-sigma)\n", runs, cfg.noise_m);
    std::printf("position RMS error   raw %.2f m -> filtered %.2f m (%.0f%% lower)\n", raw_rms / runs,
                kf_rms / runs, 100.0 * (1.0 - kf_rms / raw_rms));
    std::printf("track length error   raw %.1f%% -> filtered %.1f%%\n", 100.0 * raw_len_err / runs,
                100.0 * kf_len_err / runs);
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) return usage(), 2;
    std::string cmd = argv[1];
    if (cmd == "simulate") return cmd_simulate(argc, argv);
    if (cmd == "analyze") return cmd_analyze(argc, argv);
    if (cmd == "benchmark") return cmd_benchmark(argc, argv);
    usage();
    return 2;
}
