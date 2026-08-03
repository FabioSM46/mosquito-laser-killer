# AGENTS.md — Mosquito Laser Killer Architecture & Safety Enforcement

## 1. System Overview

This project implements a stereoscopic laser-targeting system for in-flight pest control. A Raspberry Pi 5 running Raspberry Pi OS (64-bit, arm64, non-RTOS Linux kernel) controls two OV9281 global-shutter cameras, two dual-channel 12-bit MCP4922 DACs driving the X/Y galvo axes, and a 2.5W Class 4 blue laser via TTL GPIO. The intended hardware design requires a DC-rated lever arm switch, a two-contact mushroom E-stop breaking a **latching safety contactor's coil circuit** (§4.8), and a positive-opening enclosure door interlock (§4.5b); the contact arrangements/ratings of the switches currently on hand have not yet been verified, and the contactor, isolator and door switch are not on hand at all.

**Critical domain constraint:** A 2.5W Class 4 laser causes instantaneous, irreversible blindness and fire hazard. Every safety guard is **structurally enforced in code** — never documented as comments or convention.

> The exact reported stock list and unresolved hardware-fit blockers live in [`docs/HARDWARE_INVENTORY.md`](docs/HARDWARE_INVENTORY.md). Full galvanometer / camera / laser parameter tables and the derived engagement envelope live in [`docs/HARDWARE_PARAMETERS.md`](docs/HARDWARE_PARAMETERS.md). Runtime values are validated at startup by `validate_engagement_volume()` (`src/safety/config_validator.cpp`).

### 1.1 Hardware Bill of Materials

| Component | Specification | Purpose |
|-----------|-------------|---------|
| Host | Raspberry Pi 5 | Real-time control and stereo vision processing |
| Cameras | 2× OV9281 global-shutter monochrome USB3 UVC | Stereoscopic target detection; reported on hand |
| Laser | 2.5 W, 450 nm blue, 33 × 70 mm; 12 V external constant-current drive; forced-air cooling; 3-pin TTL/PWM | Class 4 target laser; reported on hand, but pinout/logic thresholds are unverified |
| Test laser | 5 mW, 12 mm module | Reported on hand; wavelength, class label, supply, pinout, and TTL compatibility are unverified |
| Laser power supply | Mean Well LRS-50-12 12 VDC / 4.2 A / 50 W | Laser driver power |
| Galvo scanner + driver | 20 kpps X-Y; 400–700 nm; head ±12 V; driver ±15 VDC with ±5 V analog input at 0.33 V/° | Laser beam steering; reported on hand |
| Galvo power supply | Regulated bipolar ±15 VDC, sized for both axes | **Required but not reported in inventory**; the 12 V laser PSU is not a substitute |
| X-axis DAC | MCP4922 DIP-14 12-bit dual DAC | Differential X-axis galvo drive |
| Y-axis DAC | MCP4922 DIP-14 12-bit dual DAC | Differential Y-axis galvo drive |
| Level translation | 2 × **SN74AHCT125N**, PDIP-14 quad bus buffer — package #1 carries the four SPI signals, package #2 carries GPIO 18 alone | Specified 2026-07-29; **on order, not on hand.** `AHCT` is load-bearing: `AHC`/`HC` are pin-identical with 3.5 V thresholds. The pin-identical `AHCT126` self-gated variant was considered and rejected (`HARDWARE_WIRING.md` §9.3) |
| Monostable | 74HC123 (DIP-16), manufacturer/order code unverified | Hardware pulse-duration backstop on laser TTL (§4.1) |
| Monostable timing R | 1/2 W carbon-film 220 kΩ (1Rext/Cext → +5 V) | Reported on hand; sets one-shot period with C_ext |
| Monostable timing C | 50 V monolithic-ceramic 1 µF (across 1Cext ↔ 1Rext/Cext) | Reported on hand; nominal period ≈0.45·R·C ≈99 ms only for a matching vendor coefficient; scope verification is mandatory |
| AND gate | SN74HC08N (DIP-14) quad 2-input AND | Gates laser TTL = GPIO18 ∧ one-shot-Q (§4.1) |
| Safety contactor **K1** | 2 main poles rated for the *combined* cold-start inrush of both DC supplies, coil matched to the control circuit, ≥1 auxiliary NO | **Not on hand; required.** Does the mains switching so the mushroom breaks only the coil circuit, and latches so an E-stop *release* cannot restart the system (§4.8) |
| START button + isolator/OCP/RCD | NO momentary; upstream isolator, overcurrent protection, RCD per local rules | **Not on hand; required.** START is the deliberate restart action |
| Enclosure door interlock | Switch with 2 independent NC contacts, positive/direct opening (IEC 60947-5-1 Annex K), tool-required actuator | **Not on hand; required.** Contact 1 in the K1 coil circuit, contact 2 in series with the arm switch (§4.5b) |
| Arm switch | Lever switch; topology/DC rating unverified | Must switch laser-driver 12 V power and provide active-HIGH GPIO 24 sense |
| E-stop | Mushroom button; pole arrangement/contact ratings unverified | Two independent NC contacts: pole 1 in the K1 coil circuit, pole 2 for GPIO 25 (active LOW). A monitored-safety-relay architecture would need a third |
| Zener diodes | BZX55C3V3, DO-35, 0.5 W; count unverified | Circuit requires 2 for arm/E-stop GPIO clamps — **cathode at the sense junction, anode to GND**; reversed, each forward-clamps its node near 0.7 V |
| Reported resistors | 1/2 W carbon film: 220 kΩ and 10 kΩ; counts unverified | **6 × 10 kΩ are needed**: 1 arm series, 3 laser-path fail-LOW, 2 chip-select pull-ups |
| Missing resistors | 2× 3.3 kΩ and 1× 1 kΩ, all 1/2 W | **Must be obtained** for the arm/E-stop sense networks |
| Capacitors | 50 V monolithic ceramic: 1 µF and 100 nF; counts unverified | Timing, per-IC decoupling, and input debounce; verify sufficient quantity |
| Wiring connectors | WAGO 221-413, 3-conductor, max 4 mm²; count unverified | Power/signal distribution; one enclosed connector per electrical net |

**Power and signal wiring:**
- RPi 5 GPIO 18 → **SN74AHCT125N #2** (`/OE` tied LOW) → **74HC123 monostable + SN74HC08N AND gate** → verified laser TTL-switch input (configurable via `laser_pin`). **Three** 10 kΩ fail-LOW pull-downs, one per node: at the translator input, at its output beside the '123/'08 inputs, and at the laser-driver connector. Each covers a case none of the others reaches; the third is the only coverage for an unpowered or removed AND gate. The monostable is the independent hardware pulse-duration backstop; see §4.1.
- RPi 5 GPIO 24 → verified DC-rated lever arm switch (configurable via `arm_switch_pin`).
- RPi 5 GPIO 25 → verified two-contact NC mushroom E-stop (configurable via `e_stop_pin`).
- RPi 5 SPI0 MOSI, SCLK, CE0, and CE1 → qualified 3.3 V→5 V push-pull translation → the two MCP4922 DACs. Direct 3.3 V chip-select wiring is not guaranteed at a 5 V DAC supply.
- Both MCP4922 Vref pins tied to 5 V, producing a 0–5 V unipolar output per channel and a ±5 V differential swing per axis.
- Galvo driver powered by a separate bipolar ±15 VDC supply; laser driver powered by 12 V from the Mean Well supply. The ±15 V supply has not yet been reported and is a pre-power no-go item.
- The OV9281 cameras are capable of 1280×720; the system runs them at **640×400** (a validated OV9281 binned mode; 640×480 is not supported by this sensor). The built-in default is 120 FPS; the shipped `config/system_config.yaml` selects the validated 210 FPS mode. The `StereoFrame` buffers are dynamically sized so any supported mode works without code changes.

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

**The real bound is ~105ms, not 100ms.** The check only runs when the control thread runs, so the true limit is `max_pulse_duration_ms` plus one control cycle (a fixed 5ms — `k_control_period` in `control_loop.h`, deliberately **not** derived from `target_fps` for the same reason the watchdog timeout is not, §4.4) plus scheduling jitter. Quoting a flat 100ms would be a claim the software cannot make.

**Every *software* mechanism that can end a pulse runs on the control thread itself.** `enforce_max_pulse`, `execute_cycle`, `Laser::fire`'s re-entry check, the watchdog, and the E-stop poll are all on the *control thread*. If that thread stalls with the pin HIGH, none of them fires. This is why `core/print.h` is non-blocking (§4.11) — it removes the most likely way for that thread to stall — and it is why the pulse bound needs an enforcer that is *not* on that thread.

**Hardware backstop — 74HC123 retriggerable monostable on the TTL line.** A one-shot sits between the SN74AHCT125N #2 fail-LOW GPIO 18 interface and the laser driver TTL, wired so the laser TTL is `translated GPIO18 ∧ one-shot-Q`, with channel-1 A held LOW, the translated fire line driving channel-1 B and CLR/RD, and the active-HIGH Q taken from standard DIP pin 13 into the AND gate. The one-shot triggers on GPIO 18's rising edge and re-arms/reset while the fire line is LOW. Effect: a normal short pulse passes through unchanged (the AND gate follows GPIO 18), but a control thread that hangs with GPIO 18 stuck HIGH is force-cut when Q times out — **with no software path and no operator action.** This is the enforcer the software cannot be: it is the only thing that makes the pulse-duration bound independent of the control thread. For a vendor coefficient of 0.45, the reported `220 kΩ × 1 µF` timing parts give a nominal **99 ms**; the actual bound is the mandatory scope-measured assembled period because the on-hand 74HC123 manufacturer and passive tolerances are not yet recorded (see `docs/HARDWARE_WIRING.md` §11a). The 220 kΩ goes to **pin 14** (`1Rext/Cext`): the two channels are mirrored — channel 2 is `6 = 2Cext, 7 = 2Rext/Cext` and channel 1 reverses that — and a revision of the wiring guide had 14 and 15 swapped, which puts the resistor on the discharge node and makes the measured period meaningless.

The guarantee is real **only if three conditions hold, and each must be scope-verified — an unverified backstop is a comment (§4, §7):**
1. **The AND gating is present.** If Q drives the laser TTL *alone* (no AND with GPIO 18), every fire pulse is stretched to the full ~99 ms one-shot width — a hazard, not a guard. Verify: a short GPIO 18 pulse must produce an equally short laser pulse; a *stuck-HIGH* GPIO 18 must produce a single ~99 ms pulse and then stay dark.
2. **Firing is a single sustained level, never a PWM burst.** "Retriggerable" means edges *restart* the timer; repeated edges within the window would hold Q HIGH indefinitely and defeat the cap. Current firing (`fire(true)`/`fire(false)`) satisfies this — it is now a constraint the firing path must never violate.
3. **The measured period is what you intend.** At ~99 ms the one-shot sits *below* the ~105 ms real software bound, so hardware becomes the binding limit and will also clip legitimate max-length pulses (safe, and arguably desirable). If the backstop should only trip on a *failure*, raise it above the software bound (e.g. C_ext = 1.5 µF → ~130–150 ms). R/C tolerance alone can swing the period ±20%.

**Remaining residual risk.** The '123 is itself a single component; a failure of the one-shot or a broken AND gate must be considered in the FMEA. The operator interlocks remain the outer layers and are *not* on the control thread: the **arm switch** cuts 12 V to the driver (a true hardware interlock), and the **E-stop** cuts mains — but both require a human hand and neither is autonomous. The monostable is the only autonomous enforcer of the *duration* bound.

### 4.2 Firing Cooldown (10 seconds)

**Enforced by:** `FiringController::cooldown_until_` field, set to `now + 10s` after each pulse. The `may_fire()` query returns `false` while `now < cooldown_until_`. The firing path is gated by `may_fire()` — no bypass exists.

### 4.3 Motion Blanking

**Enforced by:** `FiringController::execute_cycle()`:
1. If a pulse is active, only enforce max pulse duration — **no galvo writes** while the laser is ON
2. When not firing: map the target and compare the codes against the last commanded pair. A delta beyond `k_settle_deadband_codes` is a **real re-aim**: write the DAC, stamp `galvo_command_time_`, clear settle. A delta within the deadband is the same aim — the mirrors are not moving — so nothing is written and the settle timer keeps running. `galvo_settled_` is set only once `now - galvo_command_time_` **strictly exceeds** `settle_delay_ms_`
3. Fire only when `armed_ && target_valid_ && galvo_settled_ && may_fire(now)`
4. After a pulse ends — by max duration **or** abort — clear settle/target **and `last_commanded_dac_`** before the next DAC command; dropping the cache means the next engagement always re-writes, so a deadband skip can never trust an aim an external path (watchdog zero, shutdown) may have moved

The deadband (12 codes ≈ 0.088° ≈ 1.2 mm at 0.75 m) exists because tracker output jitters at the sub-millimeter level every frame. Without it, each update reset the settle timer, and the fire gate could only pass on a control cycle that happened to receive *no* fresh command — the system fired only on scheduling beats, aimed at stale data by construction. With it, a hovering/slow target (position delta under the deadband per frame, ≲0.15 m/s) fires deterministically one settle delay after the aim stabilises; a faster target is chased continuously, which is the physically honest behaviour for a settle-gated single-shot system (lead-prediction aiming is possible future work).

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

**The enclosure door interlock rides this path, deliberately.** The door switch's second NC contact sits in series with the arm switch on the 12 V laser feed, *upstream* of the GPIO 24 sense tap (`docs/HARDWARE_WIRING.md` §5, §6a). So GPIO 24 means "the laser driver actually has power available", not merely "the lever is up", and opening the door reads as a disarm that this existing code path already handles correctly. **There is no door GPIO and no door-specific code, and that is the design, not an omission:** an interlock's job is to remove the hazard, not to inform software, and a contact that opens the 12 V feed is strictly stronger than a pin the control thread polls. Do not add an `IDoorInterlock` and a control-loop poll — it would buy a signal that cannot do anything the contacts have not already done.

### 4.6 Deterministic Initialization & RAII Shutdown

**Enforced by:**
- `Laser` constructor: `gpio_.set_direction(output)` then `gpio_.write(LOW)` — pin LOW before any other initialization
- `main()`'s local declaration order (C++ guarantees reverse-order destruction; the laser is deliberately declared *after* the galvo chain so `~Laser` runs first — the comment at the declarations in `src/main.cpp` marks this as load-bearing):
  1. `~Laser` writes GPIO LOW (via RAII)
  2. `~DifferentialGalvoDriver` / `~MCP4922` command mid-scale (2048) center
  3. `~SpiImpl` closes the bus handle
- `sigaction` handlers set `shutdown_requested` atomic flag; threads exit gracefully; destructors fire

### 4.7 Hardware Error Propagation

**Enforced by:** All HAL operations return `std::expected<T, HardwareError>`. Callers MUST handle the error — `[[nodiscard]]` plus `-Werror=unused-result` makes an ignored result a compile error, **in the tests too** (`tests/CMakeLists.txt` must never relax it; when it did, 16 discarded safety results went unnoticed, two of them in tests whose only call was a dropped `execute_cycle()`).

**Propagation to SAFE_HALT:** `FiringController` deliberately holds no `SystemStateMachine` — it is a control component, not a safety authority. On a hardware fault it latches itself off (`force_laser_off_and_halt`) and exposes `is_halted()`. `control_step()` polls that every cycle and drives the transition. Without that poll the loop spins forever with the laser dead while the operator's readout still reads ARMED — which is what happened before, because the function named `..._and_halt` had no way to halt anything.

### 4.8 Hardware Emergency Stop (E-Stop)

**Enforced by:** The `EStop` class reads a dedicated GPIO input (active LOW) from the sense pole of the intended two-contact NC mushroom E-stop. The `ControlThread` checks `e_stop.is_pressed()` every cycle before any arm/fire logic. If pressed, it calls `FiringController::emergency_stop()` to force the laser off and transitions the state machine to `SAFE_HALT`, then breaks the control loop. The E-stop is independent of the arm switch and the watchdog, and it bypasses all state transitions via the `ANY → SAFE_HALT` path. GPIO read failure forces **pressed** (fail-safe). The software behaviour does not prove that the on-hand button has the required two contacts or mains rating; those remain pre-power hardware checks in `docs/HARDWARE_INVENTORY.md`.

**The mushroom does not switch mains. A latching contactor does, and the reason is a software property.** `docs/HARDWARE_WIRING.md` §3 puts a safety contactor K1 in the mains path with a seal-in circuit: the mushroom breaks only K1's coil circuit, and K1's own auxiliary NO contact holds the coil energised, so **releasing the mushroom restores nothing** — only a deliberate START press does. The earlier direct-switching arrangement failed for two independent reasons, and the second is this code's own doing:

1. The LRS-50-12 specifies a 45 A cold-start inrush at 230 VAC. A pilot-duty mushroom contact subjected to that **welds**, leaving an E-stop that feels normal and disconnects nothing.
2. **`SAFE_HALT` is terminal**, so the documented recovery from an E-stop is "restart the process". With mains passing through the NC contact, an operator who releases the mushroom and restarts the software while the arm switch is still ON takes the state machine `INIT → IDLE → ARMED → TRACKING → FIRING` with no deliberate start action anywhere in the sequence. The terminal-`SAFE_HALT` design is correct; it is precisely *why* the hardware must not re-energise on release.

The Raspberry Pi is deliberately **not** on the contactor — it runs from its own USB-C supply, so the log, the GPIO 25 sense circuit and the 5 V logic rail holding the §9.3 pull-downs defined all survive an E-stop. Residual risk: a welded K1 main pole means the coil drops and the supplies stay live. Mitigating that needs a monitored safety relay with mirror-contact feedback (IEC 60947-4-1) and is an open functional-safety item, not a wiring detail.

### 4.9 Signal Shutdown

**Enforced by:** `SignalHandler` installs async-signal-safe handlers that only set an atomic flag (`is_shutdown_requested()`). All three worker threads poll this flag (and `g_shutdown_requested`) each cycle. Callbacks are **not** invoked from the signal context. Control-thread exit always runs `laser->emergency_shutdown()` and `galvo->zero()`.

### 4.10 Config Engagement Validation

**Enforced by:** `validate_engagement_volume()` at startup. Critical findings **abort** process start; non-critical findings log warnings only.

**Critical:** box beyond galvo cone; non-finite or inverted galvo limits; galvo limits beyond DAC voltage budget; non-positive stereo baseline/focal length; non-positive `dac_reference_voltage`; non-finite bounding-box coordinates; `max_pulse_duration_ms` outside (0, 100]; `cooldown_seconds` < 1.0; `settle_delay_ms` outside [0.5, 50]; `watchdog_timeout_ms` outside [5, 500]; `watchdog_startup_grace_ms` outside [100, 60000]; non-positive `target_fps` or frame dimensions; principal point outside the frame; invalid detection thresholds/areas/tolerances; `background_learning_rate` outside [0, 0.5]; `motion_threshold` outside [1, 254]; `size_tolerance_factor` outside [1, 100]; invalid `tracking.*` bounds (`confirm_hits` ∉ [1, 60], `association_gate_m` ∉ (0, 1], empty/negative speed window, `max_tracks` ∉ [1, 256]).

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

**Motion gate (background model).** When `background_learning_rate > 0`, each `Detector` keeps a running background model (`cv::accumulateWeighted`, diff computed *before* the model absorbs the current frame), and a blob must also differ from it by `motion_threshold`. Static glints, fixtures, and lens dust merge into the model and stop being reported — the single biggest discriminator between a mosquito and a bright speck. The deliberate trade: **the system engages flying targets only** — a target that lands fades from the motion mask. `background_learning_rate: 0` disables the gate (legacy bright-blob behaviour); an out-of-range rate sanitizes to disabled.

**Depth-consistent size gate.** After triangulation the matcher knows z, so a `target_size_m` object projects to a known pixel area. A blob whose area falls outside `[expected/k, expected*k]` (`size_tolerance_factor`) is a glint, a fixture, or a different animal — rejected even when epipolar, disparity, and area-ratio gates all pass. `target_size_m: 0` disables the check; a NaN or sub-1.0 tolerance sanitizes to the strictest band.

**Validated multi-target correspondence (`StereoMatcher::match_all`).** A blob pair is a candidate only if it satisfies the epipolar constraint (`|v_left − v_right| <= epipolar_tolerance_px` — for a rectified pair the same object lands on the same row, so a vertical offset *proves* they are different objects), falls inside the disparity window implied by the bounding-box z range (`d = f·b/z`), has a plausible area ratio, and passes the size gate. Exclusivity is enforced **per cluster**: any blob participating in more than one candidate pair is ambiguous, and every pair it touches is void — while a clean pair elsewhere in the same frame still yields its target. A wrong guess aims a Class 4 beam at a point never verified to hold a target, so ambiguity is always resolved toward silence. `match()` retains the single-target contract (exactly one survivor or nullopt).

**Fail-closed at the boundary.** `triangulate` rejects non-finite pixels itself rather than delegating validity to a distant consumer.

**Multi-track confirmation (`MultiTracker`).** Measurements are associated to tracks by mutual nearest neighbour within `association_gate_m`; a measurement that sits between two tracks binds only to its genuine nearest, and the other coasts. A track becomes *confirmed* only after `confirm_hits` **consecutive** matched frames — a coasted frame resets the consecutive count, so a phantom that flickers in and out inside the coast horizon can never accumulate its way to confirmation; once earned, confirmation is latched and survives coasting — and *engageable* only while its estimated speed stays inside `[min_speed_mps, max_speed_mps]` (~0 m/s is a glint or fixture; beyond max is a correspondence artefact). Tracks are capped at `max_tracks`: beyond it, new tracks are refused and live ones are never evicted (fail closed).

**Coasting through brief detection gaps.** A frame without a match does not drop a track: it coasts on `KalmanTracker::predict()` — a pure extrapolation from the last verified measurement, bounded by `k_max_predict_horizon_s` (100 ms). The coasted point still passes through every downstream gate (bounding box, galvo cone, DAC range), so the beam can never leave the verified safe volume on a prediction. Past the horizon the track is deleted — fail closed.

**Sticky engagement (`TargetSelector`).** One laser, one galvo, one mandatory cooldown: engagement is sequential no matter how many tracks exist. The selector holds the engaged target while it stays engageable and falls back to the nearest (smallest z) only when it is lost — re-picking every frame would ping-pong the galvo across the swarm and never settle long enough to fire. The control thread still receives exactly one `TargetCommand` per frame; the firing path is untouched by the multi-target machinery.

**Residual: stereo temporal skew.** The two cameras free-run without hardware sync and are grabbed sequentially, so a pair can be up to one frame period apart. For a laterally moving target the skew biases disparity and therefore z — Δz ≈ z·v·Δt/b, ≈4 cm at z = 1 m, v = 1 m/s, Δt = 4.8 ms, b = 0.12 m — and the epipolar gate only catches the *vertical* component of the motion. Per-camera driver timestamps are recorded in `StereoFrame` (`left_timestamp`/`right_timestamp`, from V4L2 `buf.timestamp` when the driver stamps CLOCK_MONOTONIC) and the processing thread logs a skew watermark every 512 frames, so a degrading rig is visible in the record. The bounding-box z margins must absorb the residual. A hard skew gate was considered and deliberately not enabled: with equal free-running frame periods it would reject roughly half of all pairs.

---

## 5. C++23 Mandatory Features

| Feature | Usage |
|---------|-------|
| `std::format` + custom `println` (`core/print.h`) | All console logging via the non-blocking logger (see §4.11). `std::println`/`std::print` are deliberately NOT used — they offer no way to decline to block. No `std::cout` anywhere |
| `std::expected<T, E>` | All hardware operations return expected; no exceptions for hardware |
| `std::optional` + monadic ops | Target detection pipeline: `.and_then()`, `.transform()`, `.or_else()` |
| `std::jthread` | All three worker threads; auto-join on destruction |

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
| `ICamera` | `MockCamera` | `capture_step()`: frame assembly, per-camera buffer identity and timestamps, left-before-right grab order, capture failure → error propagation (the halt wiring on that error stays in `main`, which may only set atomics). `CameraImpl` itself has no seam below real V4L2; its exposure-unit conversion is unit-tested in `test_camera_impl.cpp` |
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

All three thread bodies are now extracted step functions — `capture_step()` (`src/hal/capture_step.cpp`), `processing_step()` (`src/vision/processing_step.cpp`) and `control_step()` (`src/control/control_loop.cpp`) — so the per-iteration logic is the real code under test, not a copy. What remains inline in `main.cpp` is deliberate plumbing only: shutdown-flag polling, queue drain/push, heartbeat stamping, the skew watermark, and pacing sleeps — exercised end-to-end by the stress tests.

### 7.1 Unit Tests (Google Test + Google Mock)

| Test Suite | Coverage |
|-----------|----------|
| `LaserSafetyTest` | Pin LOW on init/shutdown/destructor, max-pulse enforcement, emergency shutdown |
| `WatchdogTest` | Absolute timeout boundary, bounded startup grace, **sentinel vs. real heartbeat**, stale-feed rejection, latching, halt survives hardware failure |
| `ArmSwitchTest` | Debounce HIGH→armed, LOW→disarmed, glitch rejection, read failure → **disarmed** |
| `EStopTest` | Active-low debounce, press/release, read failure and uninitialised → **pressed** |
| `CoordinateMapperTest` | Bounds, galvo cone, voltage-scale DAC **rejection** (no clamp), **non-finite → `Invalid3DPoint`** (asserting the specific error, not merely that something rejected) |
| `FiringControllerTest` | Startup blanking, arm gate, max pulse (incl. late cycles), cooldown on **every** pulse-end path, motion blanking, settle + **deadband** (jitter fires, real re-aim restarts, cache dropped on pulse end), DAC-before-fire ordering, faults latch `is_halted()`, **fire-OFF failure latches on all three end paths** |
| `ControlLoopTest` | The real `control_step()`: guard **ordering** (`InSequence`-pinned enforce-before-halt, e-stop blocks a ready fire in the same cycle), e-stop/watchdog halts, arm gating, fire sequence, target-loss glue (TRACKING→ARMED+clear, FIRING→COOLDOWN+abort), cooldown exit → re-arm, fault → SAFE_HALT, fail-safe GPIO reads |
| `SystemStateMachineTest` | Valid transitions, invalid transitions rejected, SAFE_HALT irreversibility |
| `ThreadSafeQueueTest` | Concurrent push/pop, drain_all correctness |
| `DetectorTest` | **Per-blob centroids (no frame-wide phantom)**, area gates, short frame and >max_blobs → fail closed, threshold boundary |
| `MotionDetectorTest` | **Motion gate**: first frame seeds (perched target invisible), static blob fades into background, moving blob detected indefinitely, invalid rate → gate disabled |
| `StereoMatcherTest` | Triangulation, **epipolar rejection**, disparity window, **ambiguous scene → fail closed**, non-finite rejection |
| `StereoMatcherTest (match_all)` | **Two clean pairs → two targets**, ambiguous cluster void while clean pair survives, **depth-size gate** rejects over/undersized, NaN tolerance → strictest band |
| `MultiTrackerTest` | Confirmation gating (3 **consecutive** hits — a flickering phantom never confirms; the latch survives coasting), **static point confirmed but never engageable**, ID stability, coasting through gaps, horizon death, mutual-NN association, max_tracks cap, non-finite rejection, NaN-config sanitization |
| `TargetSelectorTest` | Sticky engagement vs nearer challengers, nearest-first fallback, release on non-engageable, reset |
| `KalmanTrackerTest` | **`predict()` is pure** (repeat calls identical), convergence under **noise**, covariance shrinks, prediction leads the last measurement, stale/negative dt rejected |
| `CaptureStepTest` | `capture_step()` over `MockCamera`: frame assembly with per-camera data/timestamps, **left-before-right** grab order, left failure short-circuits, right failure propagates |
| `ProcessingStepTest` | The real detect→match→track→select pipeline with the **motion gate ON**: seed frame yields nothing, a flying target is commanded on exactly the 3rd consecutive detection with sane x/y/z, an ambiguous four-blob cluster yields silence, empty frames still emit explicit no-target commands |
| `ConfigValidatorTest` | Each critical bound, incl. the ones that disable a guard from YAML |
| `ConfigLoaderTest` | Fail-closed loading (`test_config_loader.cpp`): missing file / malformed value / explicit null → error, **no partial config escapes**; absent keys keep the types.h defaults |
| `SignalHandlerTest` | SIGINT/SIGTERM set the flag, reset clears it, programmatic callback fires, **the signal context never invokes the callback**. The end-to-end signal→pin-LOW property lives in the concurrent-shutdown stress test, on a real observed pin |
| `ExposureConversionTest` | µs → V4L2 100 µs-unit conversion boundaries and the floor of 1 unit (`test_camera_impl.cpp`) |
| `PrintTest` | Non-blocking logger: full-pipe drop + counting, partial-write resync, **per-descriptor resync independence**, `log_init`/`log_shutdown` flag save-restore (incl. a forced shared stdout/stderr open file description), shutdown-time dropped-line report |
| `MCP4922Test` | Command-bit format, range rejection, **destructor re-centres both channels** (§4.6 RAII shutdown) |
| `DifferentialGalvoDriverTest` | Complementary A/B channel writes for the differential pair, init centres both axes (and fails closed on a null/uninitialised/failing DAC), out-of-range rejection, DAC-failure propagation, `zero()` centres at midpoint |

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
├── docs/
│   ├── HARDWARE_INVENTORY.md    # Reported stock + unresolved hardware no-go items
│   ├── HARDWARE_PARAMETERS.md   # Component specs + derived engagement envelope
│   ├── HARDWARE_WIRING.md       # Physical wiring incl. the 74HC123 backstop (§11a),
│   │                            #   mains/contactor (§3), door interlock (§6a),
│   │                            #   build order (§17). Source of truth for diagrams/
│   ├── diagrams/                # Excalidraw sources + PNG renders, numbered in
│   │                            #   reading order; they follow HARDWARE_WIRING.md
│   ├── CALIBRATION.md           # Stereo + galvo calibration procedure
│   └── PRE_FLIGHT_CHECKLIST.md  # Build/validation steps and go/no-go criteria
├── src/
│   ├── main.cpp                 # Entry point, thread orchestration
│   ├── core/
│   │   ├── types.h              # Common types: Point3D, StereoFrame, TargetCommand
│   │   ├── error.h              # HardwareError enum, MappingError enum
│   │   ├── config_loader.h/.cpp # Fail-closed YAML loading (any parse error aborts)
│   │   ├── thread_safe_queue.h  # Lock-protected SPSC/MPSC queue with drain_all
│   │   └── print.h              # Non-blocking, best-effort logging (see 4.11)
│   ├── hal/
│   │   ├── igpio.h              # GPIO interface (pure virtual)
│   │   ├── ispi.h               # SPI interface (pure virtual)
│   │   ├── icamera.h            # Camera interface (pure virtual)
│   │   ├── idac.h               # DAC interface (pure virtual)
│   │   ├── ilaser.h             # Laser interface (pure virtual)
│   │   ├── igalvo_driver.h      # Galvo driver interface (pure virtual)
│   │   ├── gpio_impl.h/.cpp     # Raspberry Pi GPIO via sysfs/libgpiod
│   │   ├── spi_impl.h/.cpp      # Linux SPI via spidev
│   │   ├── camera_impl.h/.cpp   # OV9281 via V4L2
│   │   ├── capture_step.h/.cpp  # One capture-thread iteration (testable, §7)
│   │   ├── mcp4922.h/.cpp       # MCP4922 DAC via SPI
│   │   ├── differential_galvo_driver.h/.cpp  # ±5V differential galvo drive over the DAC pair
│   │   └── laser.h/.cpp         # Laser TTL control with safety timers
│   ├── safety/
│   │   ├── system_state.h/.cpp  # SystemState enum + SystemStateMachine
│   │   ├── watchdog.h/.cpp      # Heartbeat watchdog
│   │   ├── bounding_box.h/.cpp  # 3D geometric safety zone
│   │   ├── arm_switch.h/.cpp    # Arm switch input with debounce
│   │   ├── e_stop.h/.cpp        # Mushroom E-stop input with debounce
│   │   ├── signal_handler.h/.cpp # Async-signal-safe SIGINT/SIGTERM flag (§4.9)
│   │   └── config_validator.h/.cpp # Startup engagement validation (§4.10)
│   ├── vision/
│   │   ├── detector.h/.cpp      # Per-blob connected-component detection with motion gate
│   │   ├── stereo_matcher.h/.cpp # Epipolar-gated multi-target correspondence + triangulation
│   │   ├── tracker.h/.cpp       # Kalman filter tracker
│   │   ├── multi_tracker.h/.cpp # Multi-track association, confirmation, coasting
│   │   ├── target_selector.h/.cpp # Sticky single-target engagement policy
│   │   └── processing_step.h/.cpp # One processing-thread iteration (testable, §7)
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
│   │   ├── test_multi_tracker.cpp
│   │   ├── test_target_selector.cpp
│   │   ├── test_kalman_tracker.cpp
│   │   ├── test_differential_galvo_driver.cpp
│   │   ├── test_mcp4922.cpp
│   │   ├── test_camera_impl.cpp       # exposure-unit conversion (V4L2 100 µs units)
│   │   ├── test_capture_step.cpp      # capture-thread iteration over MockCamera
│   │   ├── test_processing_step.cpp   # detect→match→track→select, end to end
│   │   ├── test_config_loader.cpp     # fail-closed YAML loading
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
- Compile flags: `-Wall -Wextra -Wpedantic -Werror -Werror=unused-result -Wno-unused-parameter` — **including the tests** (`tests/CMakeLists.txt` must not relax `-Werror=unused-result`; see §4.7). `-Wno-unused-parameter` is the one deliberate `-Wextra` carve-out: interface stubs and gmock overrides leave parameters unused by design, and `-Werror` would otherwise reject them
- Libraries: `mosquito_hal` → `mosquito_safety` → `mosquito_control`, plus `mosquito_vision`. Tests link these libraries rather than re-listing `src/*.cpp`, so a test can never link a different build of a safety component than the binary ships.
- Architecture-specific tuning: `-march=native` — automatically targets the host CPU's full instruction set (arm64 NEON/v8 on RPi 5) without hardcoding architecture names
- Release build: `-O3 -DNDEBUG` — aggressive optimization, assertions stripped
- Debug build: `-O0 -g3` — no optimization, full debug symbols
- Dependencies: `libgpiod` + `libgpiodcxx` (gpiod character device API, C++ bindings), `OpenCV 4.5+` (stereo, Kalman; the Raspberry Pi OS Bookworm deployment target ships 4.6), `Eigen3` (linear algebra), `yaml-cpp` (config parsing)
- Test dependencies: `GTest`, `GMock`

---

## 10. Design Assumptions & Constraints

1. **Raspberry Pi OS (64-bit, arm64) on Raspberry Pi 5** — all paths, bus topology, and hardware assumptions target this platform
2. **Linux only** — uses `/dev/spidev*`, `/dev/gpiochip*`, `/dev/video*`
3. **Non-RTOS** — worst-case scheduling latency ~10ms; watchdog tolerance accounts for this
4. **Multi-target tracking, sequential engagement** — the pipeline holds many tracks at once, but one laser and one galvo mean engagement is one target at a time, chosen by the sticky `TargetSelector` (§4.12)
5. **Indoor/controlled lighting** — detection assumes controlled background; outdoor use requires retuning
6. **Fixed camera baseline** — stereo calibration is loaded at startup; no online recalibration
7. **No persistence to disk** — state is ephemeral; no recovery on restart except config reload
8. **Camera identification via stable by-path symlinks** — `/dev/v4l/by-path/` symlinks are tied to physical USB port topology, not enumeration order. This is critical: swapping left/right cameras corrupts stereo disparity and would aim the laser at incorrect 3D positions
9. **Camera mode 640×400** — the OV9281 hardware supports 1280×720; 640×400 is a validated OV9281 binned mode (640×480 is not supported). The built-in default is 120 FPS; the shipped `config/system_config.yaml` selects the validated 210 FPS mode. The `StereoFrame` buffers are dynamically sized (`std::vector`) so any supported mode works without code changes. `target_fps` is a performance knob only: the watchdog timeout and the fixed 5 ms control period are independent of it, and the validator requires the frame period to fit inside the watchdog timeout.

---

## 11. Communication Protocols

For the physical wiring corresponding to these protocols, see `docs/HARDWARE_WIRING.md`.

- **SPI:** Mode 0, 20 MHz (MCP4922 max). Two MCP4922 dual-channel DACs on Bus 0: CS0 for the X-axis DAC, CS1 for the Y-axis DAC. Within each DAC, channel A is the positive side and channel B is the inverted side of the differential pair, producing a true ±5 V swing.
- **TTL Laser:** GPIO 18 (configurable via `laser_pin`) via libgpiod C++ character device API (`/dev/gpiochip0`), through **SN74AHCT125N #2** (`/OE` tied LOW) and the 74HC123/74HC08 backstop to the verified laser TTL-switch input, with the three fail-LOW pull-downs of `docs/HARDWARE_WIRING.md` §9.3. The driver's TTL polarity must be confirmed **active HIGH** before connection: on an active-LOW input every one of those pull-downs becomes a fire command.
- **Arm Switch:** GPIO 24 (configurable via `arm_switch_pin`), active HIGH. The same lever switch must switch 12 V power to the laser driver as a hardware interlock; verify its topology and DC rating. The enclosure door interlock is in series with it and the sense tap is downstream of both, so an open door reads as a disarm (§4.5b).
- **E-Stop:** GPIO 25 (configurable via `e_stop_pin`), active LOW. Two independent NC contacts: pole 1 breaks the **K1 contactor coil circuit** — not the mains itself — and pole 2 drives the GPIO sense circuit. K1 latches, so releasing the mushroom does not restore power (§4.8). The on-hand button must be verified against that requirement.
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
