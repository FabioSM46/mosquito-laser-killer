# AGENTS.md — Mosquito Laser Killer Architecture & Safety Enforcement

## 1. System Overview

This project implements a stereoscopic laser-targeting system for in-flight pest control. A Raspberry Pi 5 running standard (non-RTOS) Linux controls two OV9281 global-shutter cameras, a dual-channel 12-bit DAC (MCP4922) driving galvo mirrors, and a 2.5W Class 4 blue laser via TTL GPIO.

**Critical domain constraint:** A 2.5W Class 4 laser causes instantaneous, irreversible blindness and fire hazard. Every safety guard is **structurally enforced in code** — never documented as comments or convention.

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
- `IDLE → ARMED`: User/external arm command received
- `ARMED → TRACKING`: Valid target detected within bounding box
- `TRACKING → FIRING`: Galvo settled at target coordinates, cooldown expired
- `FIRING → COOLDOWN`: Pulse complete or max pulse duration exceeded
- `COOLDOWN → IDLE`: 10-second cooldown elapsed
- `ANY → SAFE_HALT`: Watchdog timeout, bounds violation, hardware error, signal received

No transition from `SAFE_HALT` back to any operational state — requires full system restart.

---

## 4. Safety Guards — Structural Enforcement

### 4.1 Laser Pulse Duration Limit (≤100ms)

**Enforced by:** `FiringController` owns a `std::chrono::steady_clock::time_point` tracking pulse start. On every SPI cycle (~8.3ms), a check `(now - pulse_start) > 100ms` forces `laser.write(false)`. There is no code path that can hold the pin HIGH for longer — the check happens in the same function that writes the pin.

### 4.2 Firing Cooldown (10 seconds)

**Enforced by:** `FiringController::cooldown_until_` field, set to `now + 10s` after each pulse. The `may_fire()` query returns `false` while `now < cooldown_until_`. The firing path is gated by `may_fire()` — no bypass exists.

### 4.3 Motion Blanking

**Enforced by:** `FiringController` tracks `galvo_settled_` flag. The execution order in `ControlThread::run()` is:
1. Write new DAC values for target position
2. Wait `settle_delay_ms_` (configurable, default 3ms)
3. Set `galvo_settled_ = true`
4. Only then evaluate `may_fire() && target_valid_ && galvo_settled_`
5. **After** pulse, set `galvo_settled_ = false` before next DAC write

The laser write path is dead code when `galvo_settled_ == false`.

### 4.4 Software Watchdog

**Enforced by:** `Watchdog` class in the Control Thread reads `heartbeat_atomic_`. If three consecutive cycles pass without an updated heartbeat, `SystemStateMachine::transition(SAFE_HALT)` is called, which:
1. Forces laser GPIO LOW via `Laser::emergency_shutdown()`
2. Commands DAC to (0,0) center
3. Sets internal state to `SAFE_HALT`
4. Logs the event with timestamp

**Tolerance:** 3 missed frames × 8.3ms ≈ 25ms accounts for non-RTOS scheduling jitter.

### 4.5 Coordinate Bounds Checking

**Enforced by:** `CoordinateMapper::map_to_dac()` returns `std::expected<std::pair<uint16_t, uint16_t>, MappingError>`. The validation chain:
1. Check 3D point against `BoundingBox3D` (safe firing volume)
2. Convert to angles, verify within galvo mechanical limits
3. Convert to DAC integers, verify 0–4095 range
4. If any step fails, return `std::unexpected(error)` → no DAC write occurs

### 4.6 Deterministic Initialization & RAII Shutdown

**Enforced by:**
- `Laser` constructor: `gpio_.set_direction(output)` then `gpio_.write(LOW)` — pin LOW before any other initialization
- `SystemController` destructor order (C++ guarantees reverse declaration order):
  1. `laser_` destructor writes GPIO LOW (via RAII)
  2. `dac_` destructor commands (0,0)
  3. `spi_` handle closed
- `sigaction` handlers set `shutdown_requested` atomic flag; threads exit gracefully; destructors fire

### 4.7 Hardware Error Propagation

**Enforced by:** All HAL operations return `std::expected<T, HardwareError>`. Callers MUST handle the error — the `std::expected` API forces explicit `.value()` or `.and_then()` calls. Unchecked errors are a compile-time warning (via `-Werror=unused-result`). Failures propagate to `SAFE_HALT` transition.

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
| `IDac` | `MockDac` | Motion blanking ordering — DAC write before laser fire |
| `ILaser` | `MockLaser` | Cooldown enforcement; max pulse duration; emergency shutdown |

---

## 7. Testing Plan

### 7.1 Unit Tests (Google Test + Google Mock)

| Test Suite | Coverage |
|-----------|----------|
| `LaserSafetyTest` | Pin LOW on init, pulse ≤100ms, cooldown enforced, emergency shutdown |
| `WatchdogTest` | Heartbeat timeout detection, SAFE_HALT transition, tolerance for 3 missed cycles |
| `CoordinateMapperTest` | Bounds validation, bounding box rejection, DAC range clamping |
| `FiringControllerTest` | Motion blanking ordering, cooldown gating, state transitions |
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
│   │   └── bounding_box.h       # 3D geometric safety zone
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
│   │   └── mock_laser.h
│   └── unit/
│       ├── test_safety_guards.cpp
│       ├── test_watchdog.cpp
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
- Dependencies: `libgpiod`, `OpenCV 4.8+` (stereo, Kalman), `Eigen3` (linear algebra), `yaml-cpp` (config parsing)
- Test dependencies: `GTest`, `GMock`

---

## 10. Design Assumptions & Constraints

1. **Linux only** — uses `/dev/spidev*`, `/dev/gpiochip*`, `/dev/video*`
2. **Non-RTOS** — worst-case scheduling latency ~10ms; watchdog tolerance accounts for this
3. **Single target** — the system tracks one mosquito at a time; multi-target is future scope
4. **Indoor/controlled lighting** — detection assumes controlled background; outdoor use requires retuning
5. **Fixed camera baseline** — stereo calibration is loaded at startup; no online recalibration
6. **No persistence to disk** — state is ephemeral; no recovery on restart except config reload

---

## 11. Communication Protocols

- **SPI:** Mode 0, 20 MHz (MCP4922 max), CS0 on Bus 0
- **TTL Laser:** GPIO 18, 3.3V logic → 5V level shifter → laser driver
- **Cameras:** USB 3.0 UVC, YUYV→grayscale conversion, 640×480@120fps
- **Config:** YAML file loaded at startup; bounding box, settle delays, PID gains

---

## 12. Coding Standards (Non-Negotiable)

- No raw `new`/`delete` — `std::unique_ptr`, `std::make_unique` only
- No `std::cout` — `std::println` / `std::print` only
- No exceptions for hardware errors — `std::expected` only
- No raw loops over `std::optional` chains — monadic operations only
- No polling without timeout — all waits have bounded duration
- All safety-critical branches have `else` arms that default to SAFE_HALT
- RAII for all resources: files, GPIO pins, SPI bus, memory
