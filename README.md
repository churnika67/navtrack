# navtrack

A small GPS navigation toolkit in **C and C++**. It reads NMEA 0183 data the way a
handheld GPS receiver would (bytes arriving over a serial port), checks every
sentence, turns them into position fixes, smooths the track with a Kalman filter and
reports distance and speed. A built-in GPS simulator generates noisy and corrupted
data so every part can be tested without hardware.

## What is in it

| Part | Language | What it does |
|------|----------|--------------|
| `c/src/nmea.c` | C99 | Parses `GGA` and `RMC` sentences from any talker (GP, GN, ...). Validates framing, the XOR checksum, and every field (time, lat/lon range, hemisphere, date). Returns an error code instead of guessing. |
| `c/src/ring_buffer.c` | C99 | Fixed-size byte ring buffer, like a UART receive queue. Extracts complete lines, counts dropped bytes, and resyncs after garbage. |
| `c/src/geo.c` | C99 | Great-circle distance (haversine), bearing, and destination point. |
| `cpp/src/decoder.cpp` | C++17 | `NmeaStreamDecoder` feeds bytes through the ring buffer and pairs GGA + RMC with the same timestamp into one `Fix`. Keeps error statistics. |
| `cpp/src/kalman.cpp` | C++17 | `KalmanSmoother`: constant-velocity Kalman filter on a local east/north plane, with measurement noise taken from HDOP. |
| `cpp/src/track.cpp` | C++17 | `Track`: distance, duration, average/max speed, GPX export. |
| `cpp/src/simulator.cpp` | C++17 | Drives a receiver along a route at 1 Hz, adds Gaussian position noise, and can flip bytes to mimic serial bit errors. |

The C core uses no heap allocation and no C++ features, so it could run on a microcontroller.
The C++ layer wraps it with classes for the host-side tools.

## Build and test

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Everything builds with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.
CI runs the tests on GCC and Clang, under Valgrind, and under AddressSanitizer + UBSan
(`-DNAVTRACK_SANITIZE=ON`).

## Use it

```bash
# 10 minutes of 1 Hz data around Cary, NC, with 5% of sentences corrupted
./build/navtrack simulate drive.nmea --seconds 600 --corrupt 0.05

# Decode it 17 bytes at a time (like a slow serial link), filter, and export GPX
./build/navtrack analyze drive.nmea --chunk 17 --gpx drive.gpx

# Compare raw vs. filtered fixes against the simulator's ground truth
./build/navtrack benchmark
```

Example output:

```
sentences      1202 (parsed 1142, checksum errors 60, field errors 0, unsupported 0)
fixes          599 (Kalman filtered)
distance       5597.7 m
```

```
runs                 20 x 300 s, noise 6.0 m (1-sigma)
position RMS error   raw 8.51 m -> filtered 5.85 m (31% lower)
track length error   raw 28.9% -> filtered 3.7%
```

Raw GPS noise makes a track zig-zag, so summing raw fixes overstates distance by
about 29% here. The filter brings that down to under 4%.

## Tests

- `tests/test_core.c`: checksum, framing, field validation, hemispheres, talker IDs,
  no-fix sentences, ring buffer wrap-around, overflow, line extraction and resync,
  and geo math against known distances and bearings.
- `tests/test_navtrack.cpp`: format/parse round trip, decoding one byte at a time and in
  large chunks, corrupted streams, garbage resync, Kalman error reduction, and track stats.

## Limits

- Spherical Earth model (good to about 0.5%), not WGS-84.
- Only GGA and RMC are parsed; other sentences are counted as unsupported.
- Times past midnight UTC are not unwrapped.
