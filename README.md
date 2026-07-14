# Mosquito Laser Killer

**Stereoscopic Laser-Targeting Pest Control System**

A C++23 real-time embedded system for Raspberry Pi 5 that detects, tracks, and neutralizes mosquitoes in flight using stereoscopic computer vision, kalman-filter trajectory prediction, and a galvanometer-steered Class 4 blue laser.

**WARNING: This system controls a 2.5W Class 4 laser capable of causing instant, irreversible blindness and fire. Read `AGENTS.md` before modifying any code. All safety guards are structurally enforced in the source code.**

## Quick Start

```bash
# Install dependencies
sudo apt install cmake build-essential libgpiod-dev libopencv-dev libeigen3-dev libyaml-cpp-dev libgtest-dev libgmock-dev

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Configure
cp ../config/system_config.yaml ./
# Edit system_config.yaml for your hardware (bounding box, stereo calibration, galvo limits)

# Run (requires sudo for GPIO/SPI access)
sudo ./mosquito_laser_killer
```

## Requirements

| Dependency | Version | Purpose |
|-----------|---------|---------|
| CMake | 3.25+ | Build system |
| GCC/Clang | 14+ (C++23) | Compiler |
| libgpiod | 2.x | GPIO control |
| OpenCV | 4.8+ | Image processing, stereo |
| Eigen3 | 3.4+ | Linear algebra |
| yaml-cpp | 0.7+ | Configuration file parsing |
| Google Test | 1.14+ | Unit testing |
| Google Mock | 1.14+ | Hardware mocking |

## Architecture

Three decoupled threads communicate via lock-protected queues:

- **Capture Thread** — Synchronizes dual OV9281 cameras at 120 FPS
- **Processing Thread** — Detects targets, computes stereo disparity, runs Kalman filter prediction. Drops stale frames to minimize latency
- **Control Thread** — Converts 3D coordinates to DAC values, controls galvo mirrors, fires laser with all safety guards active

## Configuration

All runtime parameters are in `config/system_config.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `settle_delay_ms` | 3.0 | Galvo mechanical settling time before laser fire |
| `max_pulse_duration_ms` | 100.0 | Hard limit on laser emission duration |
| `cooldown_seconds` | 10.0 | Mandatory cooldown after each pulse |
| `watchdog_missed_threshold` | 3 | Missed heartbeats before SAFE_HALT |
| `bounding_box` | -1..1m xyz | 3D safe firing zone |
| `galvo_limits` | ±20° | Galvanometer mechanical limits |
| `stereo` | baseline, focal, principal point | Stereo camera calibration |
| `left_camera_device` | `""` | Left camera device path (/dev/v4l/by-path/... or /dev/videoN) |
| `right_camera_device` | `""` | Right camera device path (/dev/v4l/by-path/... or /dev/videoN) |

### Camera Identification via USB Port

Plugging two identical OV9281 cameras into a Raspberry Pi 5 produces two `/dev/videoN` nodes whose numbering can change across reboots or re-plugs. Swapping left and right cameras silently corrupts stereo disparity, causing the laser to fire at incorrect 3D positions.

**Solution: `/dev/v4l/by-path/` symlinks.** Linux creates stable symlinks tied to physical USB port topology, not enumeration order.

```bash
# Discover your camera by-path symlinks
ls -l /dev/v4l/by-path/

# Example output on RPi 5 with two OV9281 on different USB ports:
# platform-3610000.usb-usb-0:1.1:1.0-video-index0 -> ../../video0
# platform-3610000.usb-usb-0:1.2:1.0-video-index0 -> ../../video2
```

To determine which symlink belongs to which physical camera:
1. Unplug one camera, run `ls -l /dev/v4l/by-path/` — the remaining symlink is the plugged-in camera
2. Physically label that camera "LEFT" or "RIGHT"
3. Copy its full by-path symlink into `config/system_config.yaml`

If `left_camera_device` or `right_camera_device` is left empty, the system falls back to `/dev/video0` and `/dev/video2` respectively.

## Safety Architecture

The system implements seven structurally-enforced safety guards:

1. **Laser pulse duration ≤ 100ms** — enforced by timer check in the same function that writes the pin
2. **10-second firing cooldown** — `may_fire()` gate with no bypass path
3. **Motion blanking** — laser TTL LOW during galvo slewing, fires only after settling
4. **Software watchdog** — 3 missed heartbeats (~25ms) triggers SAFE_HALT
5. **Coordinate bounds checking** — `std::expected` chain validates 3D point → bounding box → galvo limits → DAC range (0-4095)
6. **RAII deterministic shutdown** — destructor order guarantees laser LOW, DAC zeroed, SPI closed
7. **Hardware error propagation** — all HAL operations return `std::expected<T, HardwareError>`

See `AGENTS.md` for the full safety enforcement strategy.

## Building and Running Tests

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)

# Run all tests
ctest --output-on-failure

# Run specific test suite
./tests/test_safety_guards
./tests/test_watchdog
./tests/test_coordinate_mapper
./tests/test_firing_controller
./tests/test_system_state
./tests/test_thread_safe_queue
./tests/test_stereo_matcher
./tests/test_kalman_tracker
./tests/test_signal_handling
```

## Project Structure

```
├── AGENTS.md                    # Architecture & safety enforcement documentation
├── README.md                    # This file
├── CMakeLists.txt               # Top-level build
├── config/
│   └── system_config.yaml       # Runtime configuration
├── src/
│   ├── main.cpp                 # Entry point, thread orchestration
│   ├── core/                    # Types, errors, thread-safe queue
│   ├── hal/                     # Hardware abstraction layer
│   ├── safety/                  # State machine, watchdog, bounding box
│   ├── vision/                  # Detection, stereo matching, tracking
│   └── control/                 # Coordinate mapping, firing controller
└── tests/
    ├── CMakeLists.txt
    ├── mocks/                   # Google Mock interfaces
    └── unit/                    # Unit test suites
```

## Tuning Guidance

### Detection
- Adjust `threshold_` (default 128) in `vision/detector.h` for your lighting conditions
- Modify `min_contour_area_` (default 50) to filter noise vs. targets

### Stereo Calibration
- Update `config/system_config.yaml` stereo section with calibrated values
- Baseline: distance between camera optical centers in meters
- Focal length: from camera calibration (pixels)

### Safety Bounds
- `bounding_box.z_min`: set to distance where laser power is safe at minimum galvo angle
- `bounding_box.z_max`: set to distance where laser power remains effective
- Never expand bounds beyond the physical safe firing area

## License

Internal use only. This system controls a Class 4 laser. Unauthorized modification may result in serious injury.
