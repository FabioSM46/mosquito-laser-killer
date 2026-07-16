# AGENTS.md — Mosquito Laser Killer Architecture & Safety Enforcement

## 1. System Overview

This project implements a stereoscopic laser-targeting system for in-flight pest control. A Raspberry Pi 5 running Raspberry Pi OS (64-bit, arm64, non-RTOS Linux kernel) controls two OV9281 global-shutter cameras, a dual-channel 12-bit DAC (MCP4922) driving galvo mirrors, and a 2.5W Class 4 blue laser via TTL GPIO. A lever SPST arm switch and a mushroom DPST emergency-stop button provide hardwired operator control.

**Critical domain constraint:** A 2.5W Class 4 laser causes instantaneous, irreversible blindness and fire hazard. Every safety guard is **structurally enforced in code** — never documented as comments or convention.

> Full galvanometer / camera / laser parameter tables and the derived engagement envelope live in [`docs/HARDWARE_PARAMETERS.md`](docs/HARDWARE_PARAMETERS.md). Runtime values are validated at startup by `validate_engagement_volume()` (`src/safety/config_validator.cpp`).

### 1.1 Hardware Bill of Materials

| Component | Specification | Purpose |
|-----------|-------------|---------|
| Host | Raspberry Pi 5 | Real-time control and stereo vision processing |
| Cameras | 2× OV9281 global-shutter monochrome 720P USB3 UVC (120 FPS) | Stereoscopic target detection |
| Laser | 2.5W focusable TTL/PWM 450 nm blue Class 4 | Target neutralization |
| Laser power supply | Mean Well LRS-50-12 12 VDC / 4.2 A / 50 W | Laser driver power |
| Galvo scanner | 20 kpps 400–700 nm, powered by 15 V | Laser beam steering |
| X-axis DAC | MCP4922 DIP-14 12-bit dual DAC | Differential X-axis galvo drive |
| Y-axis DAC | MCP4922 DIP-14 12-bit dual DAC | Differential Y-axis galvo drive |
| Level shifter | 4-channel I2C/IIC bidirectional 3.3 V → 5 V | TTL level translation for laser driver |
| Arm switch | Lever SPST | System arm input (active HIGH on GPIO 24) |
| E-stop | Mushroom DPST push-button | Emergency stop (active LOW on GPIO 25) |
| Zener diode | BZX55C3V3 1/2 W | E-stop input overvoltage protection |
| Resistor | 1/2 W 3.3 kΩ | E-stop pull-up/pull-down |
| Resistors | 2× 1/2 W 10 kΩ | E-stop series/input protection |
| Capacitor | 100 nF ceramic | E-stop debounce / input filtering |

**Power and signal wiring:**
- RPi 5 GPIO 18 → level shifter → laser TTL input (configurable via `laser_pin`).
- RPi 5 GPIO 24 → lever SPST arm switch (configurable via `arm_switch_pin`).
- RPi 5 GPIO 25 → mushroom DPST E-stop (configurable via `e_stop_pin`).
- RPi 5 SPI0 CE0 (pin 24) → MCP4922 #1 `/CS` (X-axis); SPI0 CE1 (pin 26) → MCP4922 #2 `/CS` (Y-axis).
- Both MCP4922 Vref pins tied to 5 V, producing a 0–5 V unipolar output per channel and a ±5 V differential swing per axis.
- Galvo scanner powered by 15 V; laser driver powered by 12 V from the Mean Well supply.
- The OV9281 cameras are capable of 1280×720; the default configuration runs them at **640×400 @ 120 FPS** (a validated OV9281 binned mode; 640×480 is not supported by this sensor). The `StereoFrame` buffers are dynamically sized so any supported mode works without code changes.

---

## 2. Thread Architecture

### 2.1 Three-Thread Decoupled Paradigm

```
┌─────────────────┐     ┌──────────────────────┐     ┌─────────────────┐
│  CAPTURE THREAD  │────▶│  PROCESSING/CV THREAD │────▶│  CONTROL THREAD  │
│  (120 FPS sync)  │     │  (detect/track/map)   │     │  (DAC/laser/WD)  │
└─────────────────┘     └──────────────────────┘     └─────────────────┘
```

**Capture Thread → Processing Thread:** `ThreadSafeQueue<StereoFrame>`  
**Processing Thread → Control Thread:** `ThreadSafeQueue<TargetCommand>`  
**Processing Thread → Control Thread (Heartbeat):** `std::atomic<std::chrono::steady_clock::time_point>`

### 2.2 Frame Dropping (Latency Over Throughput)

The Processing Thread dequeues **all** frames at once, discarding all but the newest. If the queue accumulated N frames during processing, N-1 are dropped. This guarantees the tracking pipeline always operates on the freshest data — stale frames would cause the laser to aim at positions the target already vacated.

Implementation: `ThreadSafeQueue::drain_all()` returns all queued items; caller keeps only the last.

### 2.3 Thread Lifecycle

- All threads are `std::jthread` — the destructor auto-joins on scope exit
- A global `std::atomic<bool> shutdown_requested` coordinates graceful shutdown
- Signal handlers (`SIGINT`, `SIGTERM`) set this flag; threads check it each cycle

---

## 3. Safety State Machine

```
                    ┌──────────────────────────────────────────────┐
                    │                                              │
                    ▼                                              │
  ┌──────┐    ┌──────────┐    ┌────────┐    ┌───────────┐    ┌──────┐
  │ INIT │───▶│   IDLE   │───▶│ ARMED  │───▶│ TRACKING  │───▶│ FIRING│
  └──────┘    └──────────┘    └────────┘    └───────────┘    └──┬───┘
                   ▲                                              │
                   │               ┌───────────┐                  │
                   └───────────────│ COOLDOWN  │◀─────────────────┘
                                   └───────────┘

  ANY STATE ──(error/watchdog/timeout)──▶ SAFE_HALT
```

**Transitions:**
- `INIT → IDLE`: Hardware initialization complete, self-test passed
- `IDLE → ARMED`: Arm switch ON (debounced)
- `ARMED → TRACKING`: Valid target detected within bounding box
- `TRACKING → FIRING`: Galvo settled at target coordinates, cooldown expired
- `TRACKING → IDLE` / `ARMED → IDLE`: Arm switch OFF (disarm)
- `FIRING → COOLDOWN`: Pulse complete, abort, or max pulse duration exceeded
- `COOLDOWN → IDLE`: 10-second cooldown elapsed (re-arms to ARMED if switch still ON)
- `ANY → SAFE_HALT`: Watchdog timeout, E-stop, hardware error, capture failure, signal shutdown

No transition from `SAFE_HALT` back to any operational state — requires full system restart.

---

## 4. Safety Guards — Structural Enforcement

### 4.1 Laser Pulse Duration Limit (≤100ms)

**Enforced by:** `FiringController` owns a `std::chrono::steady_clock::time_point` tracking pulse start. On every SPI cycle (~8.3ms), a check `(now - pulse_start) > 100ms` forces `laser.write(false)`. There is no code path that can hold the pin HIGH for longer — the check happens in the same function that writes the pin.

### 4.2 Firing Cooldown (10 seconds)

**Enforced by:** `FiringController::cooldown_until_` field, set to `now + 10s` after each pulse. The `may_fire()` query returns `false` while `now < cooldown_until_`. The firing path is gated by `may_fire()` — no bypass exists.

### 4.3 Motion Blanking

**Enforced by:** `FiringController::execute_cycle()`:
1. If a pulse is active, only enforce max pulse duration — **no galvo writes** while the laser is ON
2. When not firing: write DAC for the target, wait `settle_delay_ms_`, set `galvo_settled_`
3. Fire only when `armed_ && target_valid_ && galvo_settled_ && may_fire()`
4. After pulse ends, clear settle/target before the next DAC command

The galvo command path is dead code while `pulse_active_ == true`.

### 4.4 Software Watchdog

**Enforced by:** `Watchdog` class in the Control Thread reads `heartbeat_atomic_`. If three consecutive cycles pass without an updated heartbeat, `SystemStateMachine::transition(SAFE_HALT)` is called, which:
1. Forces laser GPIO LOW via `Laser::emergency_shutdown()`
2. Commands galvos to mid-scale center (0 V differential)
3. Sets internal state to `SAFE_HALT`
4. Logs the event with timestamp

**Tolerance:** 3 missed frames × 8.3ms ≈ 25ms accounts for non-RTOS scheduling jitter.

### 4.5 Coordinate Bounds Checking

**Enforced by:** `CoordinateMapper::map_to_dac()` returns `std::expected<DacValues, MappingError>`. The validation chain:
1. Check 3D point against `BoundingBox3D` (safe firing volume)
2. Convert to angles, verify within galvo mechanical limits
3. Convert via driver scale (`θ · V/° → V_diff → DAC code`); reject if `|V_diff|` exceeds `dac_max_diff_voltage` or code is outside 0–4095 (**no silent clamp**)
4. If any step fails, return `std::unexpected(error)` → no DAC write occurs

### 4.5b Arm Switch Gating

**Enforced by:** `FiringController::set_armed(false)` disarms and clears targets; `set_target` / fire path reject when `!armed_`. Control thread calls `set_armed(arm_switch.is_armed())` every cycle. GPIO read failure forces **disarmed** (fail-safe).

### 4.6 Deterministic Initialization & RAII Shutdown

**Enforced by:**
- `Laser` constructor: `gpio_.set_direction(output)` then `gpio_.write(LOW)` — pin LOW before any other initialization
- `SystemController` destructor order (C++ guarantees reverse declaration order):
  1. `laser_` destructor writes GPIO LOW (via RAII)
  2. `dac_` destructor commands mid-scale (2048) center
  3. `spi_` handle closed
- `sigaction` handlers set `shutdown_requested` atomic flag; threads exit gracefully; destructors fire

### 4.7 Hardware Error Propagation

**Enforced by:** All HAL operations return `std::expected<T, HardwareError>`. Callers MUST handle the error — the `std::expected` API forces explicit `.value()` or `.and_then()` calls. Unchecked errors are a compile-time warning (via `-Werror=unused-result`). Failures propagate to `SAFE_HALT` transition.

### 4.8 Hardware Emergency Stop (E-Stop)

**Enforced by:** The `EStop` class reads a dedicated GPIO input (active LOW) from a mushroom DPST button. The `ControlThread` checks `e_stop.is_pressed()` every cycle before any arm/fire logic. If pressed, it calls `FiringController::emergency_stop()` to force the laser off and transitions the state machine to `SAFE_HALT`, then breaks the control loop. The E-stop is independent of the arm switch and the watchdog, and it bypasses all state transitions via the `ANY → SAFE_HALT` path. GPIO read failure forces **pressed** (fail-safe).

### 4.9 Signal Shutdown

**Enforced by:** `SignalHandler` installs async-signal-safe handlers that only set an atomic flag (`is_shutdown_requested()`). All three worker threads poll this flag (and `g_shutdown_requested`) each cycle. Callbacks are **not** invoked from the signal context. Control-thread exit always runs `laser->emergency_shutdown()` and `galvo->zero()`.

### 4.10 Config Engagement Validation

**Enforced by:** `validate_engagement_volume()` at startup. Critical findings (box beyond galvo cone, galvo limits beyond DAC voltage budget, invalid stereo/pulse limits) **abort** process start. Non-critical findings (e.g. camera FOV narrower than galvo cone) log warnings only.

---

## 5. C++23 Mandatory Features

| Feature | Usage |
|---------|-------|
| `std::println` / `std::print` | All console logging. Thread-safe, no `std::cout` anywhere |
| `std::expected<T, E>` | All hardware operations return expected; no exceptions for hardware |
| `std::optional` + monadic ops | Target detection pipeline: `.and_then()`, `.transform()`, `.or_else()` |
| `std::jthread` | All three worker threads; auto-join on destruction |
| `std::atomic_ref` | Safe access to shared state without full mutex |
| `std::move_only_function` | Callback registration for safety hooks |

---

## 6. Hardware Abstraction & Mock Architecture

### 6.1 Interface Hierarchy

Every hardware component has a pure virtual interface (`IGpio`, `ISpi`, `ICamera`, `IDac`, `ILaser`) and a concrete implementation (`GpioImpl`, `SpiImpl`, `CameraImpl`, `MCP4922`, `Laser`). This enables:

- **Unit testing with Google Mock:** Safety guards are tested by mocking hardware and verifying pin states, DAC values, and error propagation without physical hardware
- **Simulation mode:** A `SimulatedGpio` implementation allows dry-running control algorithms

### 6.2 Mock Strategy

| Component | Mock | What We Test |
|-----------|------|--------------|
| `IGpio` | `MockGpio` | Laser pin enforced LOW on init/shutdown/error; pulse duration tracking |
| `ISpi` | `MockSpi` | DAC values validated in 0–4095 range; SPI errors → SAFE_HALT |
| `ICamera` | `MockCamera` | Frame timestamps; queue behavior under backpressure |
| `IDac` | `MockDac` | DAC values validated in 0–4095 range; SPI errors → SAFE_HALT |
| `IGalvoDriver` | `MockGalvoDriver` | Motion blanking ordering — DAC write before laser fire |
| `ILaser` | `MockLaser` | Cooldown enforcement; max pulse duration; emergency shutdown |

---

## 7. Testing Plan

### 7.1 Unit Tests (Google Test + Google Mock)

| Test Suite | Coverage |
|-----------|----------|
| `LaserSafetyTest` | Pin LOW on init, pulse ≤100ms, cooldown enforced, emergency shutdown |
| `WatchdogTest` | Heartbeat timeout detection, SAFE_HALT transition, tolerance for 3 missed cycles |
| `ArmSwitchTest` | Debounce HIGH→armed, LOW→disarmed, glitch rejection, init failure |
| `EStopTest` | Active-low debounce, press/release detection, init failure |
| `CoordinateMapperTest` | Bounds validation, voltage-scale mapping, DAC range **rejection** (no clamp) |
| `FiringControllerTest` | Motion blanking (no galvo while firing), arm gate, cooldown, pulse limits |
| `ControlArmGatingTest` | TRACKING disarm → IDLE, re-arm, no fire when disarmed |
| `SystemStateMachineTest` | All valid transitions, invalid transitions rejected, SAFE_HALT irreversibility |
| `ThreadSafeQueueTest` | Concurrent push/pop, drain_all correctness, backpressure behavior |
| `StereoMatcherTest` | Disparity calculation correctness, invalid match rejection |
| `KalmanTrackerTest` | Prediction convergence, covariance updates |
| `SignalHandlerTest` | SIGINT/SIGTERM → laser LOW, DAC zeroed |

### 7.2 Stress Tests

| Test | Method |
|------|--------|
| Frame flooding | Push 10× normal frame rate into queue; verify only newest processed |
| Watchdog jitter | Delay Processing Thread by 20ms, 30ms, 50ms; verify 3-miss tolerance, SAFE_HALT on 4th |
| Concurrent shutdown | Signal while all threads active; verify laser LOW within one cycle |
| SPI backpressure | Mock SPI delays; verify queue doesn't overflow |

---

## 8. Directory Structure

```
mosquito-laser-killer/
├── AGENTS.md                    # This file — architecture & safety documentation
├── README.md                    # Quick-start, build, configuration
├── CMakeLists.txt               # Top-level build
├── config/
│   └── system_config.yaml       # Runtime configuration (bounding box, settle ms, etc.)
├── src/
│   ├── main.cpp                 # Entry point, signal handlers, thread orchestration
│   ├── core/
│   │   ├── types.h              # Common types: Point3D, StereoFrame, TargetCommand
│   │   ├── error.h              # HardwareError enum, MappingError enum
│   │   └── thread_safe_queue.h  # Lock-protected SPSC/MPSC queue with drain_all
│   ├── hal/
│   │   ├── igpio.h              # GPIO interface (pure virtual)
│   │   ├── ispi.h               # SPI interface (pure virtual)
│   │   ├── icamera.h            # Camera interface (pure virtual)
│   │   ├── idac.h               # DAC interface (pure virtual)
│   │   ├── ilaser.h             # Laser interface (pure virtual)
│   │   ├── gpio_impl.h/.cpp     # Raspberry Pi GPIO via sysfs/libgpiod
│   │   ├── spi_impl.h/.cpp      # Linux SPI via spidev
│   │   ├── camera_impl.h/.cpp   # OV9281 via V4L2
│   │   ├── mcp4922.h/.cpp       # MCP4922 DAC via SPI
│   │   └── laser.h/.cpp         # Laser TTL control with safety timers
│   ├── safety/
│   │   ├── system_state.h       # SystemState enum + SystemStateMachine
│   │   ├── watchdog.h           # Heartbeat watchdog
│   │   ├── bounding_box.h       # 3D geometric safety zone
│   │   ├── arm_switch.h/.cpp    # Arm switch input with debounce
│   │   └── e_stop.h/.cpp        # Mushroom E-stop input with debounce
│   ├── vision/
│   │   ├── detector.h/.cpp      # Mosquito detection (thresholding, morphology)
│   │   ├── stereo_matcher.h/.cpp # Block-matching stereo disparity
│   │   └── tracker.h/.cpp       # Kalman filter tracker
│   └── control/
│       ├── coordinate_mapper.h/.cpp  # 3D→DAC conversion with bounds checking
│       └── firing_controller.h/.cpp  # Laser fire sequencing with all safety gates
├── tests/
│   ├── CMakeLists.txt
│   ├── mocks/
│   │   ├── mock_gpio.h
│   │   ├── mock_spi.h
│   │   ├── mock_camera.h
│   │   ├── mock_dac.h
│   │   ├── mock_galvo_driver.h
│   │   └── mock_laser.h
│   └── unit/
│       ├── test_safety_guards.cpp
│       ├── test_watchdog.cpp
│       ├── test_arm_switch.cpp
│       ├── test_e_stop.cpp
│       ├── test_coordinate_mapper.cpp
│       ├── test_firing_controller.cpp
│       ├── test_system_state.cpp
│       ├── test_thread_safe_queue.cpp
│       ├── test_stereo_matcher.cpp
│       ├── test_kalman_tracker.cpp
│       └── test_signal_handling.cpp
└── .clang-format
```

---

## 9. Build System

- **CMake 3.25+** with `CXX_STANDARD 23`
- Compile flags: `-Wall -Wextra -Werror -Werror=unused-result -Wpedantic`
- Architecture-specific tuning: `-march=native` — automatically targets the host CPU's full instruction set (arm64 NEON/v8 on RPi 5) without hardcoding architecture names
- Release build: `-O3 -DNDEBUG` — aggressive optimization, assertions stripped
- Debug build: `-O0 -g3` — no optimization, full debug symbols
- Dependencies: `libgpiod` + `libgpiodcxx` (gpiod character device API, C++ bindings), `OpenCV 4.8+` (stereo, Kalman), `Eigen3` (linear algebra), `yaml-cpp` (config parsing)
- Test dependencies: `GTest`, `GMock`

---

## 10. Design Assumptions & Constraints

1. **Raspberry Pi OS (64-bit, arm64) on Raspberry Pi 5** — all paths, bus topology, and hardware assumptions target this platform
2. **Linux only** — uses `/dev/spidev*`, `/dev/gpiochip*`, `/dev/video*`
3. **Non-RTOS** — worst-case scheduling latency ~10ms; watchdog tolerance accounts for this
4. **Single target** — the system tracks one mosquito at a time; multi-target is future scope
5. **Indoor/controlled lighting** — detection assumes controlled background; outdoor use requires retuning
6. **Fixed camera baseline** — stereo calibration is loaded at startup; no online recalibration
7. **No persistence to disk** — state is ephemeral; no recovery on restart except config reload
8. **Camera identification via stable by-path symlinks** — `/dev/v4l/by-path/` symlinks are tied to physical USB port topology, not enumeration order. This is critical: swapping left/right cameras corrupts stereo disparity and would aim the laser at incorrect 3D positions
9. **Default camera mode 640×400@120fps** — the OV9281 hardware supports 1280×720; the default 640×400 mode is a validated OV9281 binned mode (640×480 is not supported). The `StereoFrame` buffers are dynamically sized (`std::vector`) so any supported mode works without code changes. Higher rates (up to 210 FPS at 640×400) are configurable via `target_fps`.

---

## 11. Communication Protocols

For the physical wiring corresponding to these protocols, see `docs/HARDWARE_WIRING.md`.

- **SPI:** Mode 0, 20 MHz (MCP4922 max). Two MCP4922 dual-channel DACs on Bus 0: CS0 for the X-axis DAC, CS1 for the Y-axis DAC. Within each DAC, channel A is the positive side and channel B is the inverted side of the differential pair, producing a true ±5 V swing.
- **TTL Laser:** GPIO 18 (configurable via `laser_pin`) via libgpiod C++ character device API (`/dev/gpiochip0`), 3.3 V logic → 5 V level shifter → laser driver TTL input.
- **Arm Switch:** GPIO 24 (configurable via `arm_switch_pin`), active HIGH. The same lever SPST switch also switches 12 V power to the laser driver as a hardware interlock.
- **E-Stop:** GPIO 25 (configurable via `e_stop_pin`), active LOW mushroom DPST. One pole breaks mains Live to the power supplies; the second pole drives the GPIO sense circuit for redundancy.
- **Cameras:** USB 3.0 UVC, grayscale capture, 640×400@120fps by default (configurable via `frame_width`, `frame_height`, `target_fps`; OV9281 supports up to 210 FPS at 640×400).
- **Config:** YAML file loaded at startup; bounding box, settle delays, pulse/cooldown limits, GPIO pins, camera device paths.

---

## 12. Coding Standards (Non-Negotiable)

- No raw `new`/`delete` — `std::unique_ptr`, `std::make_unique` only
- No `std::cout` — `std::println` / `std::print` only
- No exceptions for hardware errors — `std::expected` only
- No raw loops over `std::optional` chains — monadic operations only
- No polling without timeout — all waits have bounded duration
- All safety-critical branches have `else` arms that default to SAFE_HALT
- RAII for all resources: files, GPIO pins, SPI bus, memory
