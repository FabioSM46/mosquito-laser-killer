# Pre-Flight Checklist — Mosquito Laser Killer

**WARNING:** This project is designed around the reported **2.5 W Class 4 laser**
(450 nm blue, 33 × 70 mm). For all real-world testing described in this
document, that module must be **disconnected** and replaced with a verified
low-power visible laser or no laser at all. A 5 mW, 12 mm test module is on hand,
but its wavelength, supply, pinout, TTL compatibility, and labelled class have
not yet been recorded; “5 mW” alone does not make it a verified drop-in test
source. Even low-power lasers can cause eye injury. Wear appropriate laser
safety eyewear and never bypass the enclosure or E-stop.

This checklist separates what is already implemented in software from what must be physically built, calibrated, and validated before the system can reliably detect and track a target. It is intended to be followed step by step; do not proceed to the next section until every item in the current section is checked.

---

## 1. What Is Already Implemented

The following software and hardware-abstraction components are present and pass unit/stress tests:

| Area | Status | Details |
|------|--------|---------|
| Thread architecture | Implemented | Three decoupled threads: capture, processing, control. |
| Frame dropping | Implemented | `ThreadSafeQueue::drain_all()` keeps only the freshest frame. |
| Safety state machine | Implemented | `INIT → IDLE → ARMED → TRACKING → FIRING → COOLDOWN` plus `SAFE_HALT`. |
| Max pulse limit | Implemented in software; hardware pending measurement | `FiringController` / `Laser` enforce the 100 ms config limit; the true software bound is ~105 ms (limit + one fixed 5 ms control cycle + jitter). The 74HC123 independently caps the TTL at its scope-measured period; ~99 ms is only the nominal calculation for the reported 220 kΩ / 1 µF parts and a matching vendor coefficient. |
| Cooldown | Implemented | 10-second non-bypassable cooldown after each pulse. |
| Motion blanking | Implemented | No galvo writes while the laser is ON. |
| Arm switch gating | Implemented | `FiringController` rejects targets/fire when disarmed. |
| Watchdog | Implemented | Absolute 25 ms heartbeat timeout (deliberately independent of `target_fps`) with a bounded startup grace → `SAFE_HALT` and laser OFF. |
| E-Stop | Implemented in software; hardware pending verification | Active-low GPIO 25. The intended physical circuit requires two independent NC contacts, including a mains-rated disconnect pole; the on-hand mushroom button's contacts/ratings are not yet recorded. |
| Coordinate bounds | Implemented | 3D box + galvo cone + DAC voltage scale; out-of-range commands are rejected, not clamped. |
| RAII shutdown | Implemented | Laser forced LOW, galvos centered on destruction or error. |
| Signal shutdown | Implemented | SIGINT/SIGTERM poll flag in all threads. |
| Hardware abstraction | Implemented | Mockable interfaces for GPIO, SPI, DAC, laser, galvo driver. |
| Differential galvo drive | Implemented | One MCP4922 per axis; ChA positive, ChB inverted, producing true ±5 V differential. |
| Coordinate mapping | Implemented | `CoordinateMapper` converts 3D target → angles → DAC code. |
| Stereo triangulation | Implemented | `StereoMatcher` computes 3D position from left/right pixel disparity. |
| Multi-target correspondence | Implemented | `match_all()` validates every epipolar/disparity/area/size-consistent pair; ambiguous clusters fail closed while clean pairs survive. |
| Motion gate | Implemented | Per-camera background model; only moving blobs are reported (flying targets only). |
| Depth-consistent size gate | Implemented | Blobs whose area is implausible for a `target_size_m` object at the measured z are rejected. |
| Multi-track tracking | Implemented | `MultiTracker` holds many tracks with IDs, confirmation (3 consecutive hits), speed plausibility window, and 100 ms bounded coasting. |
| Sticky target selection | Implemented | `TargetSelector` engages one target until lost, then falls back to nearest. |
| Kalman tracker | Implemented | `KalmanTracker` smooths trajectories and provides predicted target position. |
| Configuration validation | Implemented | `validate_engagement_volume()` aborts startup on critical mismatches. |

---

## 2. Prerequisites Before Applying Any Power

Do not plug in the 230 V AC until every item below is complete. Note that most of
these items do not need mains at all: `HARDWARE_WIRING.md` §17 stages 0–5 run
entirely on SELV, because the 5 V logic rail comes from the Pi header. Build and
verify the logic, backstop and sense circuits first; the mains package is stage 6.

- [ ] Every no-go item in [`HARDWARE_INVENTORY.md`](HARDWARE_INVENTORY.md) is
  closed and the actual fitted part numbers/quantities are recorded.
- [ ] Enclosure is built and fully closed with no laser exit path except the intended beam aperture.
- [ ] Beam dump or laser-absorbing backstop is installed inside the enclosure.
- [ ] **The mains switching architecture of `HARDWARE_WIRING.md` §3 is built and
  commissioned by a qualified person.** Mains does not pass through the mushroom
  contact: a safety contactor K1 does the switching, both live conductors are
  broken, and the mushroom breaks only the coil circuit. Commission by measuring:
  - Closing the isolator leaves K1 de-energised — both DC rails at 0 V.
  - START energises K1 and it seals in when START is released.
  - E-stop pressed → K1 drops → **both DC rails measured at 0 V**, not assumed.
  - **E-stop released → nothing happens.** Only a START press restores power. If
    releasing the mushroom re-energises the supplies, this is a **no-go**.
- [ ] The mushroom assembly is verified to provide two independent NC contacts:
  pole 1 in the K1 coil circuit, pole 2 driving GPIO 25. Pole 1 must be rated for
  the coil current.
- [ ] **The enclosure door interlock of `HARDWARE_WIRING.md` §6a is fitted and
  tested.** Two independent positive-opening NC contacts:
  - Door opened → K1 drops → both DC rails at 0 V; closing the door alone does
    not restore power, START is required.
  - Door opened → laser-driver +VIN removed on its own path, and GPIO 24 reads
    disarmed.
  - The actuator cannot be held closed by hand without a tool.
- [ ] **The galvo driver's own manual confirms all three points in
  `HARDWARE_WIRING.md` §10**: `IN+`/`IN−` are a genuine differential pair, the
  permitted input common-mode range includes the **2.5 V** the complementary DAC
  pair presents, and the ±5 V figure is a *differential* rating. If any is
  unconfirmed, **no-go** — the failure is a silent scale/offset error in where the
  beam points.
- [ ] The laser cooling fan is wired **upstream** of the arm switch, so it runs
  whenever the 12 V branch is live rather than stopping the instant the operator
  disarms a module that was just dissipating 2.5 W.
- [ ] The laser driver's TTL input is confirmed **active HIGH** from its own
  documentation. On an active-LOW input every fail-LOW pull-down in §9.3 becomes a
  fire command: **no-go** until the polarity is known.
- [ ] The lever switch's contact arrangement and DC rating are verified; it
  switches 12 V to the laser driver **and** feeds the GPIO 24 sensing circuit.
- [ ] A correctly rated bipolar ±15 VDC supply is installed for the galvo
  driver. The Mean Well LRS-50-12 remains dedicated to the 12 V laser branch.
- [ ] The GPIO sense networks contain meter-verified **3.3 kΩ** (×2), 10 kΩ, and
  **1 kΩ** resistors, and in each network the 3.3 kΩ, the 100 nF, the Zener and the
  GPIO wire all meet at **one junction** with the series resistor. Without that
  connection there is no divider — and the pin still reads HIGH, so the omission is
  silent. The stocked 3.3 Ω parts are segregated and not fitted.
- [ ] Both BZX55C3V3 clamps are fitted **cathode (banded end) at the sense
  junction, anode to GND**, confirmed with a diode test. Reversed, each one
  forward-clamps its node near 0.7 V.
- [ ] The full 10 kΩ count is present: **6** — one arm series, three laser-path
  fail-LOW pull-downs, two chip-select pull-ups (`HARDWARE_WIRING.md` §2).
- [ ] The four SPI outputs use a push-pull translator verified at
  `spi_speed_hz`; GPIO 18 uses a separate `AHCT125` package verified fail-LOW
  through power-up/down, open-wire, **and missing-IC** cases. The generic I2C
  module is not accepted by part name alone. All **three** pull-downs of §9.3 are
  fitted and each is individually proven:
  - Disconnect the GPIO 18 wire → laser TTL LOW. *(tests (a))*
  - Pull the `AHCT125` #2 → laser TTL LOW. *(tests (b))*
  - Pull the `SN74HC08N` → voltage at the **laser-driver connector** stays LOW.
    *(tests (c) — nothing upstream of the AND gate covers this)*
- [ ] Both MCP4922s have **`/LDAC` (pin 8) tied to GND** and **`/SHDN` (pin 9) to
  +5 V**. With `/LDAC` floating, `MCP4922::write()` returns success and the SPI
  traffic scopes correctly while the outputs never move.
- [ ] `dac_reference_voltage` holds the **measured** 5 V rail voltage at the DAC
  pin under load, not the nominal 5.0.
- [ ] E-Stop and arm switch have been tested with a multimeter, **each node
  measured with its GPIO wire disconnected before the Pi is attached**:
  - E-Stop released → ≈ 2.5 V (HIGH). If you measure ~0.8 V here, the
    sense network was built from the old drawing with a 10 kΩ series resistor
    from 3.3 V — replace it with 1 kΩ per `docs/HARDWARE_WIRING.md` §6.
  - E-Stop pressed → LOW.
  - Arm switch OFF → LOW.
  - Arm switch ON → ≈ 2.98 V (HIGH).
- [ ] **74HC123 pulse-duration backstop is wired and scope-verified** (see `docs/HARDWARE_WIRING.md` §11a). This is the only enforcer of the pulse-duration bound that is independent of the control thread:
  - The **220 kΩ is on pin 14** (`1Rext/Cext`) and the 1 µF is between pins 15 and
    14. The two channels are mirrored — channel 2 is `6 = 2Cext, 7 = 2Rext/Cext`,
    channel 1 reverses that. On pin 15 the resistor feeds the discharge node and
    the measured period means nothing: **no-go**.
  - A short (~10 ms) GPIO 18 pulse produces an equally short laser-TTL pulse (NOT stretched to ~99 ms — if stretched, the AND gate is missing/miswired: **no-go**).
  - A stuck-HIGH GPIO 18 drives the laser TTL LOW at the measured one-shot period and keeps it LOW.
  - The one-shot period is measured on a scope and **recorded**.
- [ ] All power supplies, DACs, the Pi, and the galvo driver share a single common ground.
- [ ] OD 4+ safety eyewear for 450 nm is available for every person in the room.
- [ ] The 2.5 W Class 4 laser module is **disconnected** and stored. For this checklist, use a low-power laser or no laser.

---

## 3. Calibration Required Before Aiming

The default values in `config/system_config.yaml` are placeholders. The system will **not** aim correctly until these are replaced with measured values for your specific hardware. For the step-by-step procedure behind this section (chessboard capture, intrinsic/stereo calibration, and the extrinsic galvo alignment in §5), see [`CALIBRATION.md`](CALIBRATION.md).

- [ ] Cameras are rigidly mounted with the baseline configured in `stereo.baseline_m` (default 0.12 m).
- [ ] Cameras are aimed parallel to each other (no toe-in) and aligned vertically.
- [ ] Stable USB port paths are identified with `ls -l /dev/v4l/by-path/` and copied into `left_camera_device` and `right_camera_device`.
- [ ] A chessboard stereo calibration is run to obtain:
  - `stereo.focal_length_px`
  - `stereo.cx` (≈ `frame_width / 2`)
  - `stereo.cy` (≈ `frame_height / 2`)
- [ ] `camera_optics.lens_focal_length_mm` and sensor dimensions are verified for your OV9281 module.
- [ ] The bounding box (`x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`) is set to your actual safe firing volume.
- [ ] Startup validation runs without critical errors (`[CONFIG] Aborting` does not appear).

---

## 4. Detection Validation (No Laser)

Before firing anything, confirm the vision pipeline can detect and track a moving target.

- [ ] System is powered; arm switch is OFF; E-Stop is released.
- [ ] Application starts and both cameras open successfully.
- [ ] Enclosure is dark; a bright test target (e.g., small LED on a thin wire) is moved through the bounding box (with the motion gate on, a stationary target is deliberately invisible).
- [ ] The detector finds the target in both left and right frames while it moves.
- [ ] `detection.threshold`, `min_blob_area_px`, `motion_threshold` and `background_learning_rate` in `config/system_config.yaml` are tuned so that:
  - The target is detected reliably while moving.
  - Background noise and reflections are ignored (and fade into the background model when static).
- [ ] Detection is verified with the motion gate ENABLED (`background_learning_rate > 0`): a static bright target must NOT be detected; the same target moved through the volume MUST be.
- [ ] The tracker follows the target smoothly without losing lock when the target moves.
- [ ] The processing thread reports no dropped frames when the target moves slowly.
- [ ] Dropped frames under fast target motion are reasonable (<50% at `target_fps`).
- [ ] The projected target position in the camera coordinate frame looks correct when the target is at known distances (e.g., 0.5 m, 0.75 m, 1.0 m).

---

## 5. Low-Power Laser Testing Procedure

Only proceed after Section 4 is fully validated. Replace the Class 4 laser with
a verified low-power visible alignment module. The reported 5 mW, 12 mm module
may be used only after its wavelength, labelled class, electrical supply,
pinout, and modulation behaviour are recorded and shown compatible with the
same TTL/arm/E-stop chain.

- [ ] Class 4 laser is disconnected; the verified low-power laser is installed
  with the same mounting and alignment as the eventual Class 4 laser.
- [ ] Low-power laser still requires the arm switch ON **and** GPIO 18 HIGH to emit.
- [ ] With arm switch OFF, confirm no beam is emitted.
- [ ] Swing the test target slowly through the center of the bounding box and arm the system (a confirmed track needs ~3 consecutive frames of motion).
- [ ] Verify the galvo points at the target and the low-power laser beam lands on the target.
- [ ] Move the target slowly and verify the beam follows it.
- [ ] Move the target to the edges of the bounding box (within ±15° cone) and verify the beam still tracks.
- [ ] If the beam misses consistently, check:
  - Camera calibration numbers (`focal_length_px`, `baseline_m`, `cx`, `cy`).
  - Left/right camera assignment (are they swapped?).
  - Galvo driver input scale (`galvo_driver.input_scale_v_per_deg`).
  - Physical alignment between the camera optical axis and the galvo neutral axis.
- [ ] Disarm the system and confirm the beam turns off immediately.
- [ ] Press the E-Stop and confirm the low-power laser turns off and the system halts.

---

## 6. Known Limitations

These are current limitations in the code that will affect real-world mosquito targeting. They are not bugs; they are features that may need to be upgraded depending on test results.

| Limitation | Impact | Mitigation |
|------------|--------|------------|
| **Flying targets only** | The motion gate (`background_learning_rate > 0`) merges anything stationary into the background model: a perched mosquito is invisible. This is deliberate — a static glint and a perched insect are indistinguishable at this resolution. | Accept the trade, or set `background_learning_rate: 0` to disable the gate (legacy bright-blob behaviour, far more false positives). |
| **Lighting flicker pollutes the motion mask** | 100/120 Hz mains lighting beats against the frame rate and appears as whole-frame motion. | Use a DC-driven LED backlight inside the enclosure and locked exposure (`exposure_auto: 1`, as shipped). |
| **Engagement is sequential** | One laser, one galvo, one 10 s cooldown: at most 6 shots/min regardless of how many targets are tracked. The tracker holds the swarm; the selector picks them off nearest-first. | None in software — this is hardware physics. |
| **Small target size** | A mosquito at 1 m is approximately 3–5 pixels in a 640×400 image. Detection at that scale is sensitive to noise and motion blur. | Use a longer focal length or higher resolution (1280×720@120) if the target is too small; lower the detection distance to 0.5–0.7 m. |
| **No camera/galvo extrinsic calibration** | The code assumes the camera coordinate frame and the galvo coordinate frame are aligned. In reality, there is a rotation/translation offset between them. | The low-power laser test in Section 5 is the practical calibration step. If the beam misses consistently, this offset must be measured or corrected. |
| **No real-time galvo latency compensation** | The target is tracked on the freshest frame, but the galvo has a finite mechanical response time (`settle_delay_ms`). A very fast target may move during the settle period. | Keep `target_fps` high (210 FPS) and targets within the near half of the bounding box. |

---

## 7. Go / No-Go Criteria

Do not connect the 2.5 W Class 4 laser until all of the following are true:

| Item | Required Result |
|------|-----------------|
| Inventory reconciliation | Every no-go item in `HARDWARE_INVENTORY.md` closed; fitted parts and ratings recorded |
| Build order followed | `HARDWARE_WIRING.md` §17 stages 0–11 complete, each gate passed in order |
| Mains architecture | Contactor K1, latching START circuit, double-pole isolation, isolator/OCP/RCD built and inspected by a qualified person; **E-stop release does not restart** and both DC rails measure 0 V while pressed |
| Enclosure + door interlock | Enclosure closed with beam dump; two positive-opening NC contacts fitted and both paths tested (§6a) |
| E-Stop + arm switch | Physically installed and functionally tested; sense networks metered before connection |
| Galvo driver input topology | Differential pair, 2.5 V common mode and differential ±5 V rating all confirmed from the driver's own manual (§10) |
| Power and logic interfaces | Bipolar ±15 V galvo supply installed; SPI and laser level interfaces electrically and scope verified; all three §9.3 pull-downs individually proven |
| MCP4922 control pins | `/LDAC` grounded, `/SHDN` at +5 V, `dac_reference_voltage` set from the measured rail |
| 74HC123 pulse-duration backstop | Wired with AND gating, **220 kΩ on pin 14**, and scope-verified (short pulse passes, stuck-HIGH capped, period recorded) |
| Laser driver TTL polarity | Confirmed **active HIGH** from the module's own documentation |
| Camera calibration | Real values entered, startup validation passes |
| Camera by-path identification | Stable symlinks in `config/system_config.yaml` |
| Dry-run tracking | Low-power target tracked smoothly across the bounding box |
| Low-power laser test | Beam lands on target at multiple positions |
| Safety eyewear | OD 4+ @ 450 nm available for all operators |

If any item is missing, **no-go**. Fix the issue and repeat the relevant section before proceeding.

---

## 8. After This Checklist

Once all go/no-go criteria are satisfied, the system is ready for further development or, if you choose to proceed, installation of the Class 4 laser. Connecting the Class 4 laser requires its own final checklist: alignment verification, beam dump verification, and operator training, plus the electrical, laser-safety and functional-safety review of `HARDWARE_WIRING.md` §17 stage 12. That is outside the scope of this document and must be written specifically for your jurisdiction and safety procedures.

The door interlock is no longer an open item here: it is specified as a circuit in
`HARDWARE_WIRING.md` §6a and tested in §2 above. Earlier revisions of this
checklist called for door-interlock *testing* while no circuit existed anywhere in
the documentation.
