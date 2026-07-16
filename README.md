# Mosquito Laser Killer

**Stereoscopic Laser-Targeting Pest Control System**

A C++23 real-time embedded system for Raspberry Pi 5 that detects, tracks, and neutralizes mosquitoes in flight using stereoscopic computer vision, kalman-filter trajectory prediction, and a galvanometer-steered Class 4 blue laser.

**WARNING: This system controls a 2.5W Class 4 laser capable of causing instant, irreversible blindness and fire. Read `AGENTS.md` before modifying any code. All safety guards are structurally enforced in the source code.**

## Hardware Bill of Materials

| Component | Specification | Purpose |
|-----------|-------------|---------|
| Host | Raspberry Pi 5 | Real-time control and stereo vision processing |
| Cameras | 2× OV9281 global-shutter monochrome 720P USB3 UVC (120 FPS) | Stereoscopic target detection |
| Laser | 2.5W focusable TTL/PWM 450 nm blue Class 4 | Target neutralization |
| Laser power supply | Mean Well LRS-50-12 12 VDC / 4.2 A / 50 W | Laser driver power |
| Galvo scanner | 20 kpps 400–700 nm, powered by 15 V | Laser beam steering |
| X-axis DAC | MCP4922 DIP-14 12-bit dual DAC | Differential X-axis galvo drive |
| Y-axis DAC | MCP4922 DIP-14 12-bit dual DAC | Differential Y-axis galvo drive |
| Level shifter | 4-channel I2C/IIC bidirectional 3.3 V → 5 V | 3.3 V → 5 V level translation for SPI and laser TTL |
| Arm switch | Lever SPST | System arm input (active HIGH) |
| E-stop | Mushroom DPST push-button | Emergency stop (active LOW) |
| Zener diode | BZX55C3V3 1/2 W | E-stop and arm-switch input overvoltage protection |
| Resistor | 1/2 W 3.3 kΩ | E-stop and arm-switch pull-down / pull-up |
| Resistors | 3× 1/2 W 10 kΩ | E-stop series (×2) and arm-switch series (×1) input protection |
| Capacitor | 100 nF ceramic | E-stop and arm-switch debounce / input filtering |

### Wiring Notes

A complete wiring guide is in [`docs/HARDWARE_WIRING.md`](docs/HARDWARE_WIRING.md). Key points:

- **RPi 5 GPIO 18** → level shifter → laser TTL input (3.3 V logic shifted to 5 V). Configurable via `laser_pin`.
- **RPi 5 GPIO 24** → lever SPST arm switch sense circuit (active HIGH when armed). The same arm switch also switches 12 V power to the laser driver. Configurable via `arm_switch_pin`.
- **RPi 5 GPIO 25** → mushroom DPST E-stop sense (active LOW when pressed). One E-stop pole breaks mains Live; the second pole drives the GPIO sense circuit. Configurable via `e_stop_pin`.
- **RPi 5 SPI0 CE0** (pin 24) → MCP4922 #1 `/CS` (X-axis DAC).
- **RPi 5 SPI0 CE1** (pin 26) → MCP4922 #2 `/CS` (Y-axis DAC).
- **RPi 5 SPI0 MOSI** (pin 19) and **SCLK** (pin 23) → level shifter → both MCP4922 SDI and SCK.
- Both MCP4922 Vref pins tied to **5 V** → 0–5 V per channel, combined as **±5 V differential** per axis. See the guide for the channel mapping and differential-drive explanation.
- Galvo scanner powered by **±15 V**; driver accepts the ±5 V differential signal from the DAC pair.
- Laser driver powered by **12 V** from the Mean Well supply through the arm switch.

For the complete E-Stop, arm switch, GPIO input, and SPI wiring details, see [`docs/HARDWARE_WIRING.md`](docs/HARDWARE_WIRING.md).

### Camera Resolution Note

The OV9281 modules are capable of 1280×720. The default configuration runs them at **640×400 @ 120 FPS** for real-time dual-camera processing on the Raspberry Pi 5. (640×480 is not a supported OV9281 mode.) Adjust `frame_width`, `frame_height`, and `target_fps` in `config/system_config.yaml` if your UVC firmware supports a different mode, but do not exceed the real-time processing budget.

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
| `laser_pin` | 18 | GPIO pin for laser TTL output (3.3 V, level-shifted to 5 V) |
| `arm_switch_pin` | 24 | GPIO pin for arm switch (reads HIGH when armed) |
| `e_stop_pin` | 25 | GPIO pin for mushroom E-stop (active LOW when pressed) |
| `frame_width` | 640 | Capture frame width (OV9281 default mode) |
| `frame_height` | 480 | Capture frame height |
| `target_fps` | 120 | Camera frame rate |
| `spi_device_x` | `/dev/spidev0.0` | X-axis DAC SPI device |
| `spi_device_y` | `/dev/spidev0.1` | Y-axis DAC SPI device |
| `spi_speed_hz` | 20'000'000 | SPI clock (20 MHz, MCP4922 max) |
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
# platform-1f00100000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0 -> ../../video0
# platform-1f00100000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0 -> ../../video2
```

To determine which symlink belongs to which physical camera:
1. Unplug one camera, run `ls -l /dev/v4l/by-path/` — the remaining symlink is the plugged-in camera
2. Physically label that camera "LEFT" or "RIGHT"
3. Copy its full by-path symlink into `config/system_config.yaml`

If `left_camera_device` or `right_camera_device` is left empty, the system falls back to `/dev/video0` and `/dev/video2` respectively.

## Safety Architecture

The system implements structurally-enforced safety guards (see `AGENTS.md` for full detail):

1. **Laser pulse duration ≤ 100ms** — control-loop + HAL max-pulse enforcement
2. **10-second firing cooldown** — `may_fire()` gate with no bypass path
3. **Motion blanking** — no galvo writes while laser ON; fire only after settle
4. **Arm switch gating** — targets/fire rejected when disarmed; GPIO fault → disarmed
5. **Software watchdog** — 3 missed heartbeats (~25ms) triggers SAFE_HALT
6. **Coordinate bounds** — box + galvo cone + voltage-scale DAC (**reject**, no clamp)
7. **E-stop** — active-low mushroom → SAFE_HALT; GPIO fault → pressed
8. **Config validation** — critical engagement mismatches abort startup
9. **RAII shutdown** — destructor order guarantees laser LOW, galvos centered
10. **Signal shutdown** — SIGINT/SIGTERM polled by all worker threads

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
./tests/test_arm_switch
./tests/test_e_stop
./tests/test_coordinate_mapper
./tests/test_firing_controller
./tests/test_control_arm_gating
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
│   ├── safety/                  # State machine, watchdog, bounding box, arm switch, E-stop
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
