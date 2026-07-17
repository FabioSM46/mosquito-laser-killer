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

The Control Thread does the same with `TargetCommand`s, for the same reason: a queued command aims at a position the target has already left. It previously used a blocking `pop(16ms)`, which took the *oldest* command and let the controller lag arbitrarily far behind after any stall — and, because that blocking pop sat between the max-pulse checks, it widened the true pulse bound to ~116ms.

Implementation: `ThreadSafeQueue::drain_all()` returns all queued items; caller keeps only the last. It never blocks, so the control loop's period is set by its own pacing sleep.

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
- `ANY → SAFE_HALT`: Watchdog timeout, E-stop, control-thread hardware error

Capture-thread failures and signal shutdown (SIGINT/SIGTERM) do NOT route through the state machine: the capture thread may only set atomics, and the signal handler only sets the shutdown flag. Both paths stop the threads and make the laser safe via the control-thread exit path (`laser->emergency_shutdown()`, `galvo->zero()`) plus RAII destructors, exiting with a non-zero code (`hardware_init_failed` for a capture fault) rather than entering SAFE_HALT.

No transition from `SAFE_HALT` back to any operational state — requires full system restart.

---

## 4. Safety Guards — Structural Enforcement

### 4.1 Laser Pulse Duration Limit (≤100ms)

**Enforced by:** `FiringController::execute_cycle()` checks `(now - pulse_start_) >= max_pulse_ms_` before anything else and forces `laser.fire(false)`. `Laser::enforce_max_pulse()` repeats the check at the HAL level, independently of the sequencer, and `control_step()` calls it first thing every cycle. `Laser::fire()` additionally re-checks on re-entry.

**The real bound is ~105ms, not 100ms.** The check only runs when the control thread runs, so the true limit is `max_pulse_duration_ms` plus one control cycle (~4.8ms at 210fps) plus scheduling jitter. Quoting a flat 100ms would be a claim the software cannot make.

**Residual risk — no hardware backstop.** Every mechanism that can end a pulse (`enforce_max_pulse`, `execute_cycle`, `Laser::fire`'s re-entry check, the watchdog, the E-stop) runs on the *control thread itself*. There is no `/dev/watchdog`, monostable, or independent timer behind the GPIO. If that thread stalls with the pin HIGH, no software path turns the laser off; recovery is the operator opening the arm switch, which is a true hardware interlock (it cuts 12V to the driver — see `docs/HARDWARE_WIRING.md`). The E-stop is *not* a substitute here: its GPIO is polled by the very thread that would be hung.

This is why `core/print.h` is non-blocking (§4.11) — it removes the most likely way for that thread to stall. **A retriggerable monostable on the laser TTL line remains the recommended hardware mitigation**; it is the only thing that would make a flat pulse-duration guarantee true.

### 4.2 Firing Cooldown (10 seconds)

**Enforced by:** `FiringController::cooldown_until_` field, set to `now + 10s` after each pulse. The `may_fire()` query returns `false` while `now < cooldown_until_`. The firing path is gated by `may_fire()` — no bypass exists.

### 4.3 Motion Blanking

**Enforced by:** `FiringController::execute_cycle()`:
1. If a pulse is active, only enforce max pulse duration — **no galvo writes** while the laser is ON
2. When not firing: write DAC for the target, stamp `galvo_command_time_`, and set `galvo_settled_` only once `now - galvo_command_time_` **strictly exceeds** `settle_delay_ms_`
3. Fire only when `armed_ && target_valid_ && galvo_settled_ && may_fire(now)`
4. After a pulse ends — by max duration **or** abort — clear settle/target before the next DAC command

The galvo command path is dead code while `pulse_active_ == true`.

The comparison in (2) is strictly `>`, not `>=`. `galvo_command_time_` is stamped in the *same* cycle the DAC is written, so with a `>=` comparison a `settle_delay_ms` of 0 marks the galvo settled at zero elapsed time and fires microseconds after the SPI write, while the mirrors are still slewing — painting the beam across the whole scan field at full power. `config_validator` also rejects `settle_delay_ms < 0.5`; the strict comparison means the controller fails closed even if that bound is ever loosened.

Settle is measured against a real deadline (`galvo_command_time_`), not an assumption that the caller sleeps.

### 4.4 Software Watchdog

**Enforced by:** `Watchdog`, polled by the Control Thread, watches the Processing Thread's heartbeat. If the heartbeat is older than `watchdog_timeout_ms`, it:
1. Forces laser GPIO LOW via `Laser::emergency_shutdown()`
2. Commands galvos to mid-scale center (0 V differential)
3. Transitions to `SAFE_HALT`

**Tolerance:** `watchdog_timeout_ms` (default 25ms), an **absolute duration**. It is deliberately *not* derived from `target_fps`: frame rate is a performance knob, and a performance knob must never retune a safety interlock. (It previously was derived, so raising `target_fps` to 210 silently cut the tolerance from 25ms to 14.3ms against a documented ~10ms worst-case scheduling latency.)

**Startup grace:** `watchdog_startup_grace_ms` (default 5s). Until the first *real* heartbeat arrives, `check()` passes — USB cameras take hundreds of ms to open. The grace is bounded: once it expires with no heartbeat, the watchdog fails closed.

**`feed()` accepts strictly newer heartbeats only.** The control thread forwards the producer's atomic every cycle whether or not the producer has run, so a value that has not advanced must not reset the timer. `main` seeds that atomic with `time_point::min()`, not `now()`; seeding it with a real timestamp made the first `feed()` look like a genuine heartbeat, which destroyed the grace and made the watchdog measure wall-time-since-launch — halting the system ~16-32ms after start, before the cameras could open, permanently (SAFE_HALT is terminal).

**Scope:** this watches the *Processing* thread. Nothing watches the Control Thread — see §4.1.

### 4.5 Coordinate Bounds Checking

**Enforced by:** `CoordinateMapper::map_to_dac()` returns `std::expected<DacValues, MappingError>`. The validation chain:
0. Reject non-finite coordinates (`std::isfinite` on x/y/z) → `Invalid3DPoint`
1. Check 3D point against `BoundingBox3D` (safe firing volume)
2. Convert to angles, verify within galvo mechanical limits
3. Convert via driver scale (`θ · V/° → V_diff → DAC code`); reject if `|V_diff|` exceeds `dac_max_diff_voltage` or code is outside 0–4095 (**no silent clamp**)
4. If any step fails, return `std::unexpected(error)` → no DAC write occurs

**Step 0 is load-bearing.** Every subsequent comparison fails *open* on NaN — `a < b` and `a > b` are both false — and `std::lround(NaN)` is unspecified (LONG_MIN here), which casts to 0 and passes the 0–4095 range check as a legitimate code: full negative deflection on both axes, reported as success. Only `BoundingBox3D::contains()` happens to catch NaN today, and that is an accident of its comparison polarity, not defense in depth.

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

**Enforced by:** All HAL operations return `std::expected<T, HardwareError>`. Callers MUST handle the error — `[[nodiscard]]` plus `-Werror=unused-result` makes an ignored result a compile error, **in the tests too** (`tests/CMakeLists.txt` must never relax it; when it did, 16 discarded safety results went unnoticed, two of them in tests whose only call was a dropped `execute_cycle()`).

**Propagation to SAFE_HALT:** `FiringController` deliberately holds no `SystemStateMachine` — it is a control component, not a safety authority. On a hardware fault it latches itself off (`force_laser_off_and_halt`) and exposes `is_halted()`. `control_step()` polls that every cycle and drives the transition. Without that poll the loop spins forever with the laser dead while the operator's readout still reads ARMED — which is what happened before, because the function named `..._and_halt` had no way to halt anything.

### 4.8 Hardware Emergency Stop (E-Stop)

**Enforced by:** The `EStop` class reads a dedicated GPIO input (active LOW) from a mushroom DPST button. The `ControlThread` checks `e_stop.is_pressed()` every cycle before any arm/fire logic. If pressed, it calls `FiringController::emergency_stop()` to force the laser off and transitions the state machine to `SAFE_HALT`, then breaks the control loop. The E-stop is independent of the arm switch and the watchdog, and it bypasses all state transitions via the `ANY → SAFE_HALT` path. GPIO read failure forces **pressed** (fail-safe).

### 4.9 Signal Shutdown

**Enforced by:** `SignalHandler` installs async-signal-safe handlers that only set an atomic flag (`is_shutdown_requested()`). All three worker threads poll this flag (and `g_shutdown_requested`) each cycle. Callbacks are **not** invoked from the signal context. Control-thread exit always runs `laser->emergency_shutdown()` and `galvo->zero()`.

### 4.10 Config Engagement Validation

**Enforced by:** `validate_engagement_volume()` at startup. Critical findings **abort** process start; non-critical findings log warnings only.

**Critical:** box beyond galvo cone; non-finite or inverted galvo limits; galvo limits beyond DAC voltage budget; non-positive stereo baseline/focal length; non-positive `dac_reference_voltage`; non-finite bounding-box coordinates; `max_pulse_duration_ms` outside (0, 100]; `cooldown_seconds` < 1.0; `settle_delay_ms` outside [0.5, 50]; `watchdog_timeout_ms` outside [5, 500]; `watchdog_startup_grace_ms` outside [100, 60000]; non-positive `target_fps` or frame dimensions; principal point outside the frame; invalid detection thresholds/areas/tolerances.

Every bound above exists because the parameter can disable a guard from YAML alone:
- `cooldown_seconds: 0` → `end_pulse` sets `cooldown_until_ = now` → re-fire within ~15ms → ~87% duty cycle, effectively CW 2.5W.
- `settle_delay_ms: 0` → galvo marked settled in the same cycle the DAC was written → fires while the mirrors slew, painting the beam across the scan field at full power.
- `target_fps: 0` → `1'000'000 / target_fps` → SIGFPE.

A safety bound enforced by a code comment is not enforced. Comparisons are phrased so a NaN from YAML falls into the reject branch.

**Non-critical:** camera FOV narrower than the galvo cone; principal point far from frame centre; `min_blob_area_px` larger than the area a `target_size_m` target projects to at `z_max` (this last one exists because the original `min_contour_area = 50` could not be met by a 5mm mosquito anywhere in the engagement volume — it admitted only objects 2-3× larger, i.e. glints and reflectors).

### 4.11 Non-Blocking Logging

**Enforced by:** `core/print.h`. `log_init()` marks stdout/stderr `O_NONBLOCK` before any thread starts; writes that would block are dropped and counted. See §4.1 for why: the control thread logs while the pin is HIGH, and a blocking write there stalls the only thread that can end a pulse. `std::println` is deliberately unused — it offers no way to decline to block.

`log_shutdown()` runs on every exit path. It reports the dropped-line count (dropped lines mean the log is an incomplete record of a run involving a Class 4 laser, which the reader must know) and **restores the original descriptor flags**. That restore matters: `O_NONBLOCK` is a property of the shared *open file description*, not of the fd, so when stdout is inherited from an interactive shell the flag is visible to the shell too, and a shell getting EAGAIN on its own stdout misbehaves.

A write truncated by `EAGAIN` leaves a fragment with no newline, so the next line is prefixed with one rather than splicing onto it and reading as a single corrupt entry.

Destructor and shutdown logs state only what was actually achieved. `~Laser` prints "pin LOW confirmed" only when the write succeeded, and reports `PIN STATE UNKNOWN` otherwise; an unconditional confirmation makes the post-incident trace assert a state that was never verified.

---

### 4.12 Target Validity — Detection and Correspondence

The aim angle is `atan2(x, z)` with `x = (u_left − cx)·z/f`, so **z cancels**: the beam direction depends only on the left pixel. z's sole job is therefore the safety discriminator — is this a mosquito at 0.7m, or a face across the room? A z computed from an unverified correspondence defeats the primary guard, so correspondence is validated rather than assumed.

**Per-blob segmentation (`Detector::detect_blobs`).** Each frame is segmented into connected components; every blob keeps its own centroid. A frame-wide centroid averages unrelated objects into a point where nothing physically exists, and that phantom passes every downstream gate: it has a clean centroid, triangulates to a real depth, and sits inside the bounding box. Worse, it sits *between* the real targets — two mosquitoes at u=200 and u=440 collapsed to a single "detection" at u=320, which at cx=320 is dead on the optical axis. Blobs outside `[min_blob_area_px, max_blob_area_px]` are dropped; a short frame or a scene with more than `max_blobs` candidates yields **nothing** (fail closed).

**Validated correspondence (`StereoMatcher::match`).** A blob pair is a candidate only if it satisfies the epipolar constraint (`|v_left − v_right| <= epipolar_tolerance_px` — for a rectified pair the same object lands on the same row, so a vertical offset *proves* they are different objects), falls inside the disparity window implied by the bounding-box z range (`d = f·b/z`), and has a plausible area ratio. **If more than one candidate survives, the result is nullopt** — an ambiguous scene is a guess, and a wrong guess aims a Class 4 beam at a point never verified to hold a target.

**Fail-closed at the boundary.** `triangulate` rejects non-finite pixels itself rather than delegating validity to a distant consumer.

**Coasting through brief detection gaps.** A single frame without a match does not drop the track: the processing thread coasts on `KalmanTracker::predict()` — a pure extrapolation from the last verified measurement, bounded by `k_max_predict_horizon_s` (100 ms). The coasted point still passes through every downstream gate (bounding box, galvo cone, DAC range), so the beam can never leave the verified safe volume on a prediction; it can only fire at a point the track extrapolates to from a target that was verified inside the box ≤100 ms earlier. Past the horizon `predict()` returns nullopt, the track is reset, and the command goes out invalid — fail closed. Resetting on every lost frame instead would discard the velocity estimate and force re-acquisition from zero on every transient occlusion.

---

## 5. C++23 Mandatory Features

| Feature | Usage |
|---------|-------|
| `std::format` + custom `println` (`core/print.h`) | All console logging via the non-blocking logger (see §4.11). `std::println`/`std::print` are deliberately NOT used — they offer no way to decline to block. No `std::cout` anywhere |
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
| `IGpio` | `MockGpio` | Laser pin enforced LOW on init/shutdown/error; **drives a real `Laser` so the pin state is observed, not asserted about a mock** |
| `ISpi` | `MockSpi` | Wire format via a real `MCP4922`; SPI errors → controller latches → SAFE_HALT |
| `ICamera` | `MockCamera` | Frame timestamps; capture failure → halt |
| `IDac` | `MockDac` | DAC values validated in 0–4095 range |
| `IGalvoDriver` | `MockGalvoDriver` | Motion blanking ordering — DAC write before laser fire |
| `ILaser` | `MockLaser` | Arm/cooldown/max-pulse gating; `enforce_max_pulse` called every cycle; emergency shutdown |

`ILaser` includes `enforce_max_pulse()` deliberately: it is a safety guard, the control loop must be able to call it on any `ILaser`, and **a guard that cannot be mocked cannot be tested**. It was previously only on the concrete `Laser`, so deleting the call from the control loop failed no test.

Prefer driving the **real** component over a mocked dependency (a real `Laser` over `MockGpio`, a real `MCP4922` over `MockSpi`) rather than mocking the component under test. A test that calls a mock and then asserts the expectation it just satisfied proves nothing.

---

## 7. Testing Plan

**The standard a safety test must meet:** *if the guard were deleted from `src/`, would this test fail?* If not, the test is decoration. This is not hypothetical — the suite once had 151 passing tests on a system that could not start, and the tests below are the ones that survived asking that question of every case.

Specifically, do not write: tests that assert on a mock the test itself called; `EXPECT_CALL` with no assertion on the outcome; tests that re-implement the logic under test (a bug in `src/` then gets mirrored into the test); or fixtures configured more permissively than production (`FiringControllerTest` once ran a ±2m box and a ±25° cone — a config this project's own `config_validator` rejects as critical).

`tests/CMakeLists.txt` must never relax `-Werror=unused-result`.

### 7.1 Unit Tests (Google Test + Google Mock)

| Test Suite | Coverage |
|-----------|----------|
| `LaserSafetyTest` | Pin LOW on init/shutdown/destructor, max-pulse enforcement, emergency shutdown |
| `WatchdogTest` | Absolute timeout boundary, bounded startup grace, **sentinel vs. real heartbeat**, stale-feed rejection, latching, halt survives hardware failure |
| `ArmSwitchTest` | Debounce HIGH→armed, LOW→disarmed, glitch rejection, read failure → **disarmed** |
| `EStopTest` | Active-low debounce, press/release, read failure and uninitialised → **pressed** |
| `CoordinateMapperTest` | Bounds, galvo cone, voltage-scale DAC **rejection** (no clamp), **non-finite → `Invalid3DPoint`** (asserting the specific error, not merely that something rejected) |
| `FiringControllerTest` | Startup blanking, arm gate, max pulse (incl. late cycles), cooldown on **every** pulse-end path, motion blanking, settle, DAC-before-fire ordering, faults latch `is_halted()` |
| `ControlLoopTest` | The real `control_step()`: guard **ordering**, e-stop/watchdog halts, arm gating, fire sequence, target-loss glue (TRACKING→ARMED+clear, FIRING→COOLDOWN+abort), cooldown exit → re-arm, fault → SAFE_HALT, fail-safe GPIO reads |
| `SystemStateMachineTest` | Valid transitions, invalid transitions rejected, SAFE_HALT irreversibility |
| `ThreadSafeQueueTest` | Concurrent push/pop, drain_all correctness |
| `DetectorTest` | **Per-blob centroids (no frame-wide phantom)**, area gates, short frame and >max_blobs → fail closed, threshold boundary |
| `StereoMatcherTest` | Triangulation, **epipolar rejection**, disparity window, **ambiguous scene → fail closed**, non-finite rejection |
| `KalmanTrackerTest` | **`predict()` is pure** (repeat calls identical), convergence under **noise**, covariance shrinks, prediction leads the last measurement, stale/negative dt rejected |
| `ConfigValidatorTest` | Each critical bound, incl. the ones that disable a guard from YAML |
| `PrintTest` | Non-blocking logger: full-pipe drop + counting, partial-write resync, `log_init`/`log_shutdown` flag save-restore (incl. shared stdout/stderr open file description) |
| `MCP4922Test` | Command-bit format, range rejection, **destructor re-centres both channels** (§4.6 RAII shutdown) |

### 7.2 Stress Tests

| Test | Method |
|------|--------|
| Frame flooding | Push 10× normal frame rate into queue; verify only newest processed |
| Watchdog jitter | **Real threads**: sustained sub-timeout jitter must not halt; a real producer stall must halt; a producer that never starts rides the grace then fails closed; concurrent stale feeds must not rewind the timer |
| Concurrent shutdown | **Real `Laser` over `MockGpio`, observing the actual pin**: get it genuinely firing, then shut down / E-stop / SIGINT; verify pin LOW, galvos zeroed, promptly |
| SPI backpressure | Real `MCP4922` over a delaying/failing `MockSpi` |

Exact timing boundaries belong in unit tests with injected time; stress tests use real threads with generous margins, so scheduler noise cannot produce a false failure.

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
│   ├── main.cpp                 # Entry point, config load, thread orchestration
│   ├── core/
│   │   ├── types.h              # Common types: Point3D, StereoFrame, TargetCommand
│   │   ├── error.h              # HardwareError enum, MappingError enum
│   │   ├── thread_safe_queue.h  # Lock-protected SPSC/MPSC queue with drain_all
│   │   └── print.h              # Non-blocking, best-effort logging (see 4.11)
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
│   │   ├── detector.h/.cpp      # Per-blob connected-component detection
│   │   ├── stereo_matcher.h/.cpp # Epipolar-gated correspondence + triangulation
│   │   └── tracker.h/.cpp       # Kalman filter tracker
│   └── control/
│       ├── coordinate_mapper.h/.cpp  # 3D→DAC conversion with bounds checking
│       ├── firing_controller.h/.cpp  # Laser fire sequencing with all safety gates
│       └── control_loop.h/.cpp       # control_step(): one control-thread iteration
├── tests/
│   ├── CMakeLists.txt
│   ├── mocks/
│   │   ├── mock_gpio.h
│   │   ├── mock_spi.h
│   │   ├── mock_camera.h
│   │   ├── mock_dac.h
│   │   ├── mock_galvo_driver.h
│   │   └── mock_laser.h
│   ├── unit/
│   │   ├── test_safety_guards.cpp
│   │   ├── test_watchdog.cpp
│   │   ├── test_arm_switch.cpp
│   │   ├── test_e_stop.cpp
│   │   ├── test_coordinate_mapper.cpp
│   │   ├── test_firing_controller.cpp
│   │   ├── test_control_loop.cpp      # exercises the real control_step()
│   │   ├── test_system_state.cpp
│   │   ├── test_thread_safe_queue.cpp
│   │   ├── test_detector.cpp
│   │   ├── test_stereo_matcher.cpp
│   │   ├── test_kalman_tracker.cpp
│   │   ├── test_config_validator.cpp
│   │   ├── test_print.cpp             # non-blocking logger (§4.11)
│   │   └── test_signal_handling.cpp
│   └── stress/
│       ├── test_frame_flooding.cpp
│       ├── test_watchdog_jitter.cpp
│       ├── test_concurrent_shutdown.cpp
│       └── test_spi_backpressure.cpp
└── .clang-format
```

---

## 9. Build System

- **CMake 3.25+** with `CXX_STANDARD 23`
- Compile flags: `-Wall -Wextra -Werror -Werror=unused-result -Wpedantic` — **including the tests** (`tests/CMakeLists.txt` must not relax `-Werror=unused-result`; see §4.7)
- Libraries: `mosquito_hal` → `mosquito_safety` → `mosquito_control`, plus `mosquito_vision`. Tests link these libraries rather than re-listing `src/*.cpp`, so a test can never link a different build of a safety component than the binary ships.
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
- No `std::cout` and no `std::println`/`std::print` — the non-blocking `println` from `core/print.h` only (see §4.11)
- No exceptions for hardware errors — `std::expected` only
- No raw loops over `std::optional` chains — monadic operations only
- No polling without timeout — all waits have bounded duration
- All safety-critical branches have `else` arms that default to SAFE_HALT
- RAII for all resources: files, GPIO pins, SPI bus, memory
