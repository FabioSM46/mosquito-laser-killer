# Mosquito Laser Killer

**Stereoscopic Laser-Targeting Pest Control System**

A C++23 real-time embedded system for Raspberry Pi 5 that detects, tracks, and neutralizes mosquitoes in flight using stereoscopic computer vision, kalman-filter trajectory prediction, and a galvanometer-steered Class 4 blue laser.

**WARNING: This system controls a 2.5W Class 4 laser capable of causing instant, irreversible blindness and fire. Read `AGENTS.md` before modifying any code. All safety guards are structurally enforced in the source code.**

## Hardware Inventory and Required Parts

The exact owner-reported inventory, including unresolved electrical and safety
fit checks, is recorded in
[`docs/HARDWARE_INVENTORY.md`](docs/HARDWARE_INVENTORY.md). “On hand” does not
mean “approved to energize”; the wiring BOM identifies additional required
parts.

| Component | Reported hardware | Status / purpose |
|-----------|-------------------|------------------|
| Host | Raspberry Pi 5 | Real-time control and stereo vision processing |
| Cameras | 2× OV9281 global-shutter monochrome USB3 UVC | On hand; stereoscopic target detection |
| Galvo scanner + driver | 20 kpps X-Y kit; 400–700 nm; head ±12 V; driver ±15 VDC, ±5 V analog input, 0.33 V/° | On hand; laser beam steering. The separate bipolar ±15 VDC supply is also on hand but unmeasured |
| Working laser | 2.5 W, 450 nm blue, 33 × 70 mm; 12 V external constant-current drive, forced-air cooling, 3-pin TTL/PWM | On hand; Class 4 target laser. Pinout and TTL levels must be verified |
| Test laser | 5 mW, 12 mm module | On hand; wavelength, supply, class label, pinout, and TTL compatibility must be recorded before use |
| Laser power supply | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W | On hand; working-laser driver power only |
| DACs | 2× MCP4922 DIP-14, dual-channel 12-bit | On hand; one DAC per galvo axis |
| Level translation | 2× **SN74AHCT125N** PDIP-14 quad bus buffer — one package for SPI, a second for GPIO 18 | On hand, but under a marketplace brand with **no manufacturer traceability**, so the `AHCT` marking certifies nothing — qualify by the §9.4 threshold test, and prefer parts from an authorised distributor. `AHC`/`HC` are pin-identical with 3.5 V thresholds |
| Pulse backstop logic | 74HC123 DIP-16 + SN74HC08N DIP-14; 220 kΩ 1/2 W + 1 µF 50 V monolithic ceramic timing parts | On hand. `74HC123` is a Hitachi HD74HC123P whose datasheet gives `t_W = R·C`, so the fitted parts are **≈220 ms**, not 99 ms. Actual period must still be measured. The 220 kΩ goes to **pin 15** |
| Safety contactor + START | 2-pole contactor rated for the *combined* cold-start inrush, ≥1 auxiliary NO, plus a START button and mains isolator/OCP/RCD | On hand; **not wired.** The mushroom must not switch mains directly |
| Door interlock | Switch with 2 independent NC contacts, positive/direct opening | On hand; contact arrangement unconfirmed |
| Switches | Lever switch + mushroom button | On hand; contact topology and ratings remain unverified |
| Input protection | BZX55C3V3 0.5 W Zeners; 100 nF 50 V monolithic ceramic capacitors | On hand; quantities not yet recorded |
| Resistor stock | 1/2 W carbon film: 220 kΩ, 10 kΩ, 3.3 kΩ, 1 kΩ | On hand; quantities and values not yet metered. **6× 10 kΩ** are needed |
| Wiring connectors | WAGO 221-413, 3-conductor | On hand; splicing connectors — not terminal blocks, and never a barrier between circuits |

**Every part is on hand; none of it is verified.** No switch contact has been
ringed out, no IC marking read, no passive metered, and the 74HC123 period has
not been measured. The safety contactor, latching START circuit, mains
isolator and door interlock are purchased but **not built**, and the galvo
driver's **differential input topology and common-mode range** are still
unconfirmed — that last one invalidates the coordinate mapping if it turns out
wrong. These are no-go items in `docs/HARDWARE_INVENTORY.md`.

### Wiring Notes

A complete wiring guide is in [`docs/HARDWARE_WIRING.md`](docs/HARDWARE_WIRING.md),
with drawings in [`docs/diagrams/`](docs/diagrams/README.md). Key points:

- **RPi 5 GPIO 18** → **SN74AHCT125N #2** (`/OE` tied LOW) → 74HC123 monostable + SN74HC08N AND-gate pulse-duration backstop → verified working-laser TTL input. Configurable via `laser_pin`. **Three** 10 kΩ fail-LOW pull-downs, one per node: translator input, translator output, and the laser-driver connector — each covers a case none of the others reaches. See AGENTS.md §4.1 and `docs/HARDWARE_WIRING.md` §9.3, §11a.
- **RPi 5 GPIO 24** → lever-switch sense circuit (active HIGH when armed). The same switch must also interrupt 12 V power to the laser driver; verify its contact arrangement and DC rating first. The door interlock is in series with it, upstream of the sense tap, so an open door reads as a disarm. Configurable via `arm_switch_pin`.
- **RPi 5 GPIO 25** → mushroom E-stop sense (active LOW when pressed). Two NC contacts are required: pole 1 breaks the **safety contactor's coil circuit** — not the mains itself — and pole 2 drives the GPIO sense circuit. The contactor latches, so releasing the mushroom does not restore power; only a deliberate START press does. Configurable via `e_stop_pin`.
- **RPi 5 SPI0 CE0** (pin 24) → MCP4922 #1 `/CS` (X-axis DAC), with a 10 kΩ pull-up to 3.3 V so both DACs stay deselected at boot.
- **RPi 5 SPI0 CE1** (pin 26) → MCP4922 #2 `/CS` (Y-axis DAC), likewise.
- **RPi 5 SPI0 MOSI, SCLK, CE0, and CE1** go through **SN74AHCT125N #1**. Direct 3.3 V chip-select wiring into a 5 V DAC is not validated.
- **Both MCP4922 `/LDAC` (pin 8) → GND and `/SHDN` (pin 9) → +5 V.** Left floating, `/LDAC` produces a silent failure: `write()` returns success, the SPI traffic scopes correctly, and the outputs never move.
- Both MCP4922 Vref pins tied to **5 V** → 0–5 V per channel, combined as **±5 V differential** per axis. `dac_reference_voltage` must hold the *measured* rail voltage.
- Galvo scanner powered by a separate **±15 V** supply (three conductors: +15 V, 0 V/COM, −15 V). Confirm from the driver's own manual that `IN+`/`IN−` are a genuine differential pair whose common-mode range admits the 2.5 V the complementary DAC pair presents, and that ±5 V is a *differential* rating.
- Laser driver powered by **12 V** through the arm switch and door interlock; the cooling fan is wired **upstream** of the arm switch so it runs whenever the 12 V branch is live.

For the complete mains, contactor, E-Stop, arm switch, door interlock, GPIO input,
and SPI wiring details — plus the risk-ordered build sequence in §17 — see
[`docs/HARDWARE_WIRING.md`](docs/HARDWARE_WIRING.md).

### Camera Resolution Note

The OV9281 modules are capable of 1280×720. The conservative built-in default is
**640×400 @ 120 FPS**; the shipped `config/system_config.yaml` selects the
previously validated **640×400 @ 210 FPS** mode. The exact on-hand module/firmware
mode list and fitted lenses still need to be recorded with `v4l2-ctl` and camera
calibration. (640×480 is not a supported OV9281 sensor mode.) Adjust
`frame_width`, `frame_height`, and `target_fps` only to a mode exposed by both
cameras and within the real-time processing budget.

## Quick Start

```bash
# Install dependencies
sudo apt install cmake build-essential libgpiod-dev libopencv-dev libeigen3-dev libyaml-cpp-dev libgtest-dev libgmock-dev

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..

# Configure
# Edit config/system_config.yaml in the repo root for your hardware
# (bounding box, stereo calibration, camera by-path symlinks, galvo limits)

# Run from the repo root (requires sudo for GPIO/SPI access). The binary loads
# config/system_config.yaml relative to the working directory, or takes the
# config path as its first argument. A config that is missing or fails to
# parse aborts startup — the system never runs on defaults.
sudo ./build/mosquito_laser_killer
# or: sudo ./build/mosquito_laser_killer /path/to/system_config.yaml
```

### Exit codes

Abnormal exits are distinguishable, so a supervisor (`systemd` with `Restart=on-failure`, a wrapper script) can tell a clean Ctrl-C from a safety abort. Anything non-zero means **review the log before restarting**.

| Code | Meaning |
|------|---------|
| 0 | Clean shutdown (SIGINT/SIGTERM) |
| 1 | Config load or validation failed — file missing/unreadable/malformed, or a parameter outside its safety bound |
| 2 | Hardware init or capture failure (GPIO, SPI, camera) |
| 3 | SAFE_HALT — a safety interlock fired (watchdog, E-stop, hardware fault while armed) |

## Requirements

| Dependency | Version | Purpose |
|-----------|---------|---------|
| CMake | 3.25+ | Build system |
| GCC 13+ / Clang 17+ | C++23 (`std::format`, `std::expected`) | Compiler — verified with GCC 13.4 |
| libgpiod | 2.x | GPIO control |
| OpenCV | 4.5+ | Image processing, stereo (Raspberry Pi OS Bookworm ships 4.6) |
| Eigen3 | 3.4+ | Linear algebra |
| yaml-cpp | 0.7+ | Configuration file parsing |
| Google Test | 1.11+ | Unit testing |
| Google Mock | 1.11+ | Hardware mocking |

## Architecture

Three decoupled threads communicate via lock-protected queues:

- **Capture Thread** — Synchronizes dual OV9281 cameras at 120 FPS
- **Processing Thread** — Detects targets, computes stereo disparity, runs Kalman filter prediction. Drops stale frames to minimize latency
- **Control Thread** — Converts 3D coordinates to DAC values, controls galvo mirrors, fires laser with all safety guards active

## Configuration

All runtime parameters are in `config/system_config.yaml`:

Every fire-control and timing parameter is range-checked at startup by `validate_engagement_volume()`; a value outside its bound **aborts the process** rather than quietly degrading a guard.

| Parameter | Default | Bound | Description |
|-----------|---------|-------|-------------|
| `settle_delay_ms` | 3.0 | [0.5, 50] | Galvo settling before fire. `0` would fire while the mirrors slew |
| `max_pulse_duration_ms` | 100.0 | (0, 100] | Hard limit on laser emission |
| `cooldown_seconds` | 10.0 | ≥ 1.0 | Mandatory cooldown. `0` gives a ~87% duty cycle, effectively CW |
| `watchdog_timeout_ms` | 25.0 | [5, 500] | **Absolute** heartbeat timeout — independent of `target_fps` |
| `watchdog_startup_grace_ms` | 5000.0 | [100, 60000] | Window for the pipeline's first heartbeat, then fails closed |
| `laser_pin` | 18 | — | GPIO pin for laser TTL output (3.3 V, level-shifted to 5 V) |
| `arm_switch_pin` | 24 | — | GPIO pin for arm switch (reads HIGH when armed) |
| `e_stop_pin` | 25 | — | GPIO pin for mushroom E-stop (active LOW when pressed) |
| `frame_width` | 640 | > 0 | Capture frame width |
| `frame_height` | 400 | > 0 | Capture frame height (OV9281 binned mode) |
| `target_fps` | 120 | > 0, frame period < watchdog timeout | Camera frame rate. A performance knob only — the watchdog timeout and the fixed 5 ms control-loop period are both independent of it |
| `spi_device_x` | `/dev/spidev0.0` | — | X-axis DAC SPI device |
| `spi_device_y` | `/dev/spidev0.1` | — | Y-axis DAC SPI device |
| `spi_speed_hz` | 20'000'000 | — | SPI clock (20 MHz, MCP4922 max) |
| `bounding_box` | x,y ±0.09m; z 0.5–1.0m | inside galvo cone | 3D safe firing zone |
| `galvo_limits` | ±15° | within DAC voltage budget | Galvanometer mechanical limits |
| `stereo` | baseline, focal, principal point | cx,cy inside frame | Stereo camera calibration |
| `detection` | threshold, blob area, epipolar tol. | see below | Target detection and stereo correspondence gates |
| `left_camera_device` | `""` | — | Left camera device path (/dev/v4l/by-path/... or /dev/videoN) |
| `right_camera_device` | `""` | — | Right camera device path (/dev/v4l/by-path/... or /dev/videoN) |

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

If either `left_camera_device` or `right_camera_device` is empty — or the two are identical — the capture thread reports the misconfiguration and the process exits with a hardware fault. There is no fallback device: guessing at `/dev/videoN` risks a swapped pair, which corrupts stereo disparity and aims the laser at wrong 3D positions.

## Safety Architecture

The system implements structurally-enforced safety guards (see `AGENTS.md` for full detail):

1. **Laser pulse duration** — control-loop + HAL max-pulse enforcement. Real software bound is ~105ms (limit + one fixed 5 ms control cycle), not a flat 100ms; the 74HC123 one-shot independently caps the TTL at its mandatory measured period (nominally ~220 ms: the on-hand Hitachi part specifies `t_W = R·C` with no coefficient, so it sits *above* the software bound)
2. **10-second firing cooldown** — `may_fire(now)` gate, applied on *every* pulse-end path (clean, aborted, and fault)
3. **Motion blanking** — no galvo writes while laser ON; fire only after settle
4. **Arm switch gating** — targets/fire rejected when disarmed; GPIO fault → disarmed
5. **Software watchdog** — absolute 25ms heartbeat timeout (not derived from `target_fps`) + bounded startup grace → SAFE_HALT
6. **Validated targeting** — per-blob detection, epipolar-gated stereo correspondence, ambiguous scene → no target
7. **Coordinate bounds** — non-finite reject + box + galvo cone + voltage-scale DAC (**reject**, no clamp)
8. **E-stop** — active-low mushroom → SAFE_HALT; GPIO fault → pressed
9. **Config validation** — any bound that could disable a guard aborts startup
10. **Fault propagation** — hardware faults latch the controller; `control_step` polls `is_halted()` → SAFE_HALT
11. **RAII shutdown** — declaration order puts `~Laser` first; shutdown logs claim only what was verified
12. **Signal shutdown** — SIGINT/SIGTERM polled by all worker threads; logging is non-blocking so the laser-owning thread cannot stall on a write

### Known residual risk

Every *software* path that can end a pulse runs on the **control thread** — if that thread stalls with the pin HIGH, no software turns the laser off, and the E-stop GPIO poll is on the same thread. That failure mode is covered by the **74HC123 retriggerable monostable + 74HC08 AND gate on the TTL line** (see the BOM above and `docs/HARDWARE_WIRING.md` §11a): once the assembled period is bench-verified, a stuck-HIGH GPIO 18 is force-cut at that measured deadline with no software or operator involvement. The residual risk is that the one-shot and AND gate are themselves single components — their wiring must be verified per `docs/PRE_FLIGHT_CHECKLIST.md` §2, and a failure of either must be considered in any FMEA. The operator interlocks stay as outer layers: the verified arm switch cuts 12 V to the driver, the door interlock cuts it independently, and the E-stop drops a latching safety contactor that removes mains from both DC supplies — all hardware mechanisms, and none of them on the control thread, but each requiring a human hand or an opened door. Because the contactor latches, releasing the mushroom restores nothing; only a deliberate START press does, which matters here because `SAFE_HALT` is terminal and the documented recovery is "restart the software". The contactor, START circuit, isolator and door interlock are specified but not yet built, and the current inventory does not establish the switch contact ratings/topology, so it is not ready for powered use.

## Building and Running Tests

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON
make -j$(nproc)

# Run all tests
ctest --output-on-failure

# Or run a specific suite from build/tests/, e.g.:
./tests/test_safety_guards
./tests/test_firing_controller
./tests/test_control_loop
```

The full set of suites is registered in `tests/CMakeLists.txt`: unit —
`test_safety_guards`, `test_watchdog`, `test_arm_switch`, `test_e_stop`,
`test_coordinate_mapper`, `test_firing_controller`, `test_system_state`,
`test_thread_safe_queue`, `test_stereo_matcher`, `test_kalman_tracker`,
`test_differential_galvo_driver`, `test_mcp4922`, `test_camera_impl`,
`test_detector`, `test_multi_tracker`, `test_target_selector`,
`test_signal_handling`, `test_config_loader`, `test_config_validator`,
`test_print`, `test_control_loop`; stress — `test_frame_flooding`,
`test_watchdog_jitter`, `test_concurrent_shutdown`, `test_spi_backpressure`.

## Project Structure

```
├── AGENTS.md                    # Architecture & safety enforcement documentation
├── README.md                    # This file
├── CMakeLists.txt               # Top-level build
├── config/
│   └── system_config.yaml       # Runtime configuration
├── docs/                        # Hardware parameters, wiring, calibration, pre-flight
├── src/
│   ├── main.cpp                 # Entry point, thread orchestration
│   ├── core/                    # Types, errors, config loader, thread-safe queue, logger
│   ├── hal/                     # Hardware abstraction layer
│   ├── safety/                  # State machine, watchdog, bounding box, arm switch,
│   │                            #   E-stop, signal handler, config validator
│   ├── vision/                  # Detection, stereo matching, multi-target tracking
│   └── control/                 # Coordinate mapping, firing controller, control_step()
└── tests/
    ├── CMakeLists.txt
    ├── mocks/                   # Google Mock interfaces
    ├── unit/                    # Unit test suites
    └── stress/                  # Real-thread stress tests (shutdown, jitter, flooding)
```

## Tuning Guidance

### Detection
All detection parameters live in the `detection:` block of `config/system_config.yaml` — none are hardcoded.
- `threshold` (default 128) for your lighting conditions
- `min_blob_area_px` / `max_blob_area_px` to separate targets from noise and from lamps/glints. The startup validator warns if `min_blob_area_px` is larger than the area your `target_size_m` actually projects to at `z_max` — i.e. if the floor is set so high that no real mosquito could pass it
- `epipolar_tolerance_px` — how far apart, vertically, two blobs may be and still be considered the same object. Depends on your rectification quality

### Stereo Calibration
- Full step-by-step procedure: [`docs/CALIBRATION.md`](docs/CALIBRATION.md) (chessboard capture, intrinsic/stereo calibration script, extrinsic galvo alignment)
- Update `config/system_config.yaml` stereo section with calibrated values
- Baseline: distance between camera optical centers in meters
- Focal length: from camera calibration (pixels)

### Safety Bounds
- `bounding_box.z_min`: set to distance where laser power is safe at minimum galvo angle
- `bounding_box.z_max`: set to distance where laser power remains effective
- Never expand bounds beyond the physical safe firing area

## License

Internal use only. This system controls a Class 4 laser. Unauthorized modification may result in serious injury.
