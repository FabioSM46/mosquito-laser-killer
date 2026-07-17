# Pre-Flight Checklist — Mosquito Laser Killer

**WARNING:** This project is designed around a **2.5 W Class 4 laser** (450 nm blue). For all real-world testing described in this document, the Class 4 laser module must be **disconnected** and replaced with a **low-power visible laser** (e.g., ≤5 mW red or green) or no laser at all. Even low-power lasers can cause eye injury; wear appropriate laser safety eyewear and never bypass the enclosure or E-stop.

This checklist separates what is already implemented in software from what must be physically built, calibrated, and validated before the system can reliably detect and track a target. It is intended to be followed step by step; do not proceed to the next section until every item in the current section is checked.

---

## 1. What Is Already Implemented

The following software and hardware-abstraction components are present and pass unit/stress tests:

| Area | Status | Details |
|------|--------|---------|
| Thread architecture | Implemented | Three decoupled threads: capture, processing, control. |
| Frame dropping | Implemented | `ThreadSafeQueue::drain_all()` keeps only the freshest frame. |
| Safety state machine | Implemented | `INIT → IDLE → ARMED → TRACKING → FIRING → COOLDOWN` plus `SAFE_HALT`. |
| Max pulse limit | Implemented | Hard-limited to 100 ms by `FiringController` / `Laser`. |
| Cooldown | Implemented | 10-second non-bypassable cooldown after each pulse. |
| Motion blanking | Implemented | No galvo writes while the laser is ON. |
| Arm switch gating | Implemented | `FiringController` rejects targets/fire when disarmed. |
| Watchdog | Implemented | 3 missed heartbeats → `SAFE_HALT` and laser OFF. |
| E-Stop | Implemented | Active-low GPIO 25 + physical mains disconnect via DPST. |
| Coordinate bounds | Implemented | 3D box + galvo cone + DAC voltage scale; out-of-range commands are rejected, not clamped. |
| RAII shutdown | Implemented | Laser forced LOW, galvos centered on destruction or error. |
| Signal shutdown | Implemented | SIGINT/SIGTERM poll flag in all threads. |
| Hardware abstraction | Implemented | Mockable interfaces for GPIO, SPI, DAC, laser, galvo driver. |
| Differential galvo drive | Implemented | One MCP4922 per axis; ChA positive, ChB inverted, producing true ±5 V differential. |
| Coordinate mapping | Implemented | `CoordinateMapper` converts 3D target → angles → DAC code. |
| Stereo triangulation | Implemented | `StereoMatcher` computes 3D position from left/right pixel disparity. |
| Kalman tracker | Implemented | `KalmanTracker` smooths trajectories and provides predicted target position. |
| Configuration validation | Implemented | `validate_engagement_volume()` aborts startup on critical mismatches. |

---

## 2. Prerequisites Before Applying Any Power

Do not plug in the 230 V AC until every item below is complete.

- [ ] Enclosure is built and fully closed with no laser exit path except the intended beam aperture.
- [ ] Beam dump or laser-absorbing backstop is installed inside the enclosure.
- [ ] Mushroom DPST E-Stop is wired: one pole breaks mains Live, the second pole drives GPIO 25.
- [ ] Arm switch is wired: switches 12 V to the laser driver **and** feeds the GPIO 24 sensing circuit.
- [ ] E-Stop and arm switch have been tested with a multimeter:
  - E-Stop released → GPIO 25 HIGH.
  - E-Stop pressed → GPIO 25 LOW.
  - Arm switch OFF → GPIO 24 LOW.
  - Arm switch ON → GPIO 24 ~2.98 V (HIGH).
- [ ] **74HC123 pulse-duration backstop is wired and scope-verified** (see `docs/HARDWARE_WIRING.md` §11a). This is the only enforcer of the pulse-duration bound that is independent of the control thread:
  - A short (~10 ms) GPIO 18 pulse produces an equally short laser-TTL pulse (NOT stretched to ~99 ms — if stretched, the AND gate is missing/miswired: **no-go**).
  - A stuck-HIGH GPIO 18 drives the laser TTL LOW at the measured one-shot period and keeps it LOW.
  - The one-shot period is measured on a scope and recorded.
- [ ] All power supplies, DACs, the Pi, and the galvo driver share a single common ground.
- [ ] OD 4+ safety eyewear for 450 nm is available for every person in the room.
- [ ] The 2.5 W Class 4 laser module is **disconnected** and stored. For this checklist, use a low-power laser or no laser.

---

## 3. Calibration Required Before Aiming

The default values in `config/system_config.yaml` are placeholders. The system will **not** aim correctly until these are replaced with measured values for your specific hardware.

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
- [ ] Enclosure is dark; a bright test target (e.g., small LED on a thin wire) is placed inside the bounding box.
- [ ] The detector finds the target in both left and right frames.
- [ ] `threshold_` and `min_contour_area_` in `vision/detector.h` are tuned so that:
  - The target is detected reliably.
  - Background noise and reflections are ignored.
- [ ] The tracker follows the target smoothly without losing lock when the target moves.
- [ ] The processing thread reports no dropped frames when the target is stationary.
- [ ] Dropped frames under fast target motion are reasonable (<50% at `target_fps`).
- [ ] The projected target position in the camera coordinate frame looks correct when the target is at known distances (e.g., 0.5 m, 0.75 m, 1.0 m).

---

## 5. Low-Power Laser Testing Procedure

Only proceed after Section 4 is fully validated. Replace the Class 4 laser with a low-power Class 2 or 3R laser module (e.g., ≤5 mW, red or green). Keep the same TTL/arm/E-stop wiring.

- [ ] Class 4 laser is disconnected; low-power laser is installed with the same mounting and alignment as the eventual Class 4 laser.
- [ ] Low-power laser still requires the arm switch ON **and** GPIO 18 HIGH to emit.
- [ ] With arm switch OFF, confirm no beam is emitted.
- [ ] Place the test target in the center of the bounding box and arm the system.
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
| **Simple threshold detector** | `Detector` counts pixels above a threshold and returns their centroid. It does not perform contour analysis, background subtraction, or object classification. | Use a dark background, bright backlight, and tune `threshold_` and `min_contour_area_`. If this fails, the detector will need a more sophisticated algorithm. |
| **No stereo correspondence matching** | `StereoMatcher` assumes the bright centroid in the left image corresponds to the bright centroid in the right image. Multiple bright spots, reflections, or noise can cause false matches. | Ensure only one bright target is in view during early tests. If multiple bright spots are present, add a correspondence matcher (e.g., epipolar-constrained template matching). |
| **Small target size** | A mosquito at 1 m is approximately 3–5 pixels in a 640×400 image. Detection at that scale is sensitive to noise and motion blur. | Use a longer focal length or higher resolution (1280×720@120) if the target is too small; lower the detection distance to 0.5–0.7 m. |
| **No camera/galvo extrinsic calibration** | The code assumes the camera coordinate frame and the galvo coordinate frame are aligned. In reality, there is a rotation/translation offset between them. | The low-power laser test in Section 5 is the practical calibration step. If the beam misses consistently, this offset must be measured or corrected. |
| **No real-time galvo latency compensation** | The target is tracked on the freshest frame, but the galvo has a finite mechanical response time (`settle_delay_ms`). A very fast target may move during the settle period. | Keep `target_fps` high (210 FPS) and targets within the near half of the bounding box. |

---

## 7. Go / No-Go Criteria

Do not connect the 2.5 W Class 4 laser until all of the following are true:

| Item | Required Result |
|------|-----------------|
| Enclosure + E-Stop + arm switch | Physically installed and functionally tested |
| 74HC123 pulse-duration backstop | Wired with AND gating and scope-verified (short pulse passes, stuck-HIGH capped) |
| Camera calibration | Real values entered, startup validation passes |
| Camera by-path identification | Stable symlinks in `config/system_config.yaml` |
| Dry-run tracking | Low-power target tracked smoothly across the bounding box |
| Low-power laser test | Beam lands on target at multiple positions |
| Safety eyewear | OD 4+ @ 450 nm available for all operators |

If any item is missing, **no-go**. Fix the issue and repeat the relevant section before proceeding.

---

## 8. After This Checklist

Once all go/no-go criteria are satisfied, the system is ready for further development or, if you choose to proceed, installation of the Class 4 laser. Connecting the Class 4 laser requires its own final checklist: alignment verification, beam dump verification, door-interlock testing, and operator training. That is outside the scope of this document and must be written specifically for your jurisdiction and safety procedures.
