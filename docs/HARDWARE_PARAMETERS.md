# Hardware Parameters — Mosquito Laser Killer

This document records the vendor-quoted parameters of the reported galvanometer,
cameras, and laser, and derives the **engagement envelope** that the software
enforces. The as-reported stock list and unresolved fit checks are in
[`HARDWARE_INVENTORY.md`](HARDWARE_INVENTORY.md). Values in this file are not
treated as measured until the pre-flight procedure records a bench result. For
physical wiring and assembly instructions, see
[`docs/HARDWARE_WIRING.md`](HARDWARE_WIRING.md). The runtime configuration lives
in `config/system_config.yaml`; a startup validator
(`src/safety/config_validator.cpp`, see `validate_engagement_volume`) checks that
the configured safe firing volume, galvo limits, and camera field of view are
mutually consistent and prints warnings on mismatch. **Critical** mismatches abort
startup.

---

## 1. 20 kpps X-Y Galvanometer + Driver

Reported product description: **“20Kpps Laser Galvo X-Y Scanning Galvanometer
SLA 3D DIY Animation Stage Light.”**

### 1.1 Galvo head

| Parameter | Value |
|-----------|-------|
| Rated scan speed | 20 kpps |
| Maximum scan angle | ±30° optical (default ±15°) |
| Mirror | 11 × 7 × 0.7 mm, dielectric film, >99% reflectivity @ 45° AoI |
| Wavelength coverage | 400–700 nm |
| Operating voltage | ±12 V (vendor rating for the head; it is driven by the ±15 VDC driver of §1.2, which supplies the rails and the coil drive) |
| Operating temperature | 0 °C to +45 °C |
| Storage temperature | −10 °C to +60 °C |
| Operating noise | ≤ 30 dB |
| Average current | 0.5 A |
| Peak current | Vendor listing says “1.5 A (1 A)”; the parenthetical value is ambiguous and must be resolved before sizing the supply |
| Linearity | 99.9% |
| Small-step response | ≤ 0.50 ms |
| Long-term drift (8 h) | proportional < 50 PPM/°C, zero < 15 µrad/°C |
| Repeatability | 8 µrad |
| Coil resistance | 4 Ω ± 10% |
| Coil inductance | 200 µH ± 10% |
| Coil temperature | ≤ 95 °C |

### 1.2 Driver

| Parameter | Value |
|-----------|-------|
| Input voltage | ±15 VDC |
| Analog signal input range | ±5 V |
| Analog signal input impedance | 200 kΩ ± 1% |
| Position signal input impedance | 1 kΩ ± 1% |
| Input position scale | 0.33 V/° |
| Output position scale | 0.33 V/° |
| Thermal drift | max 40 PPM/°C |
| Operating temperature | 0 °C to +45 °C |
| Storage temperature | −10 °C to +60 °C |

### 1.3 Electrical chain (DAC → driver → galvo)

```
MCP4922 (12-bit, 0–5 V unipolar per channel)
   └── differential pair:  ChA = V+, ChB = (4095-x) inverted  →  ±5 V swing
        └── Driver input (±5 V, 0.33 V/°)  →  ±15° optical (default)
             └── Galvo mirror  →  beam steered ±15° optical
```

The MCP4922 is a **dual-channel** DAC. Each axis consumes one entire DAC: channel A
carries the positive half of the differential signal and channel B carries the inverted
(negative) half. This is how the system generates a true ±5 V differential signal
although the Raspberry Pi itself can only output positive logic levels. See
[`docs/HARDWARE_WIRING.md`](HARDWARE_WIRING.md) for the physical wiring and the
[`CoordinateMapper`](../src/control/coordinate_mapper.cpp) source for the math.

**Channel mapping:**

| DAC | SPI chip select | Channel A | Channel B | Signal |
|-----|-----------------|-----------|-----------|--------|
| X-axis | GPIO 8 / CE0 | X+ | X- (inverted) | Horizontal mirror |
| Y-axis | GPIO 7 / CE1 | Y+ | Y- (inverted) | Vertical mirror |

**Signal map for one axis:**

| DAC code | ChA (V+) | ChB (V−) | V_diff = V+ − V− | Optical angle |
|----------|----------|-----------|------------------|---------------|
| 0 | 0 V | 4.9988 V | −4.9988 V | −15.148° |
| 2048 | 2.5000 V | 2.4988 V (code 2047) | +1.22 mV | +0.0037° |
| 4095 | 4.9988 V | 0 V | +4.9988 V | +15.148° |

**Key consequence:** the nominal ±5 V differential DAC range (actual code
endpoints ±4.9988 V), combined with the **0.33 V/°** driver input scale, commands
at most approximately **±15.15°** optical.
The galvo head can mechanically reach ±30° optical, but only ±10 V of drive would
get there — unreachable from this DAC. Therefore the software hard-limits the
galvo to **±15°** (`galvo_limits`), and the validator flags any configuration
whose half-cone exceeds `dac_max_diff_voltage / input_scale_v_per_deg`.

The MCP4922 unity-gain transfer is `V(code) = code/4096 · Vref`, and the driver
writes channel B as `4095 − c`. Therefore, at channel-A code `c` (0…4095):

`V_diff = (2·c − 4095)/4096 · 5 V`, and `θ = V_diff / 0.33`.

There is no pair of integer complementary codes with exactly equal voltage.
The commanded centre `c = 2048` produces a one-LSB differential residual
(~1.22 mV, ~0.0037°), negligible relative to the 12-code settle deadband but not
literally 0 V. Direct DAC shutdown writes 2048 to both channels and is exactly
0 V differential.

---

## 2. Cameras (OV9281, stereo pair)

Two OV9281 modules are reported on hand. The table below records the sensor and
previously selected UVC-mode assumptions; the exact module/firmware identifier,
advertised V4L2 modes, and fitted lenses must still be read from the hardware.

| Parameter | Value |
|-----------|-------|
| Sensor | OV9281, 1/4", global-shutter monochrome |
| Native array | 1280 × 800, 3 µm pixels |
| Sensor width / height | 3.84 mm × 2.4 mm |
| Interface | USB3 UVC, up to 210 FPS at 640×400 |
| High-rate modes | MJPG 1280×720@120, 640×400@210, 640×360@210 |
| Low-rate mode | YUV 1280×720@10 |
| Adjustable V4L2 controls | Brightness, Contrast, Saturation, White balance, Gamma, Sharpness, Exposure, Gain |

### 2.1 Lens design target (fitted lens unverified)

| Lens | H-FOV (½-FOV) | Distortion | Verdict for ±15° galvo cone |
|------|---------------|-----------|-----------------------------|
| 1.3 mm | ~112° (56°) | yes | too wide; fisheye breaks the pinhole stereo model |
| 2.4 mm | ~77° (39°) | yes | usable but requires an undistortion stage |
| **3 mm** | **~65° (33°)** | **free** | **design target — covers ±15° cone with margin; confirm this is actually fitted and calibrate distortion** |
| 6 mm | ~36° (18°) | free | too narrow for a ±15° cone — camera can't see galvo corners |

Field of view is derived from the physical lens (resolution/binning independent):

```
H-FOV = 2 · atan(sensor_width_mm / (2 · focal_length_mm))
V-FOV = 2 · atan(sensor_height_mm / (2 · focal_length_mm))
```

For the nominal 3 mm design target: H-FOV ≈65.2°, V-FOV ≈43.6°. Do not use
those values as as-built facts until the fitted lens and calibration results are
recorded.

### 2.2 Focal length in pixels (calibration)

`stereo.focal_length_px` is a **calibrated** quantity obtained from a chessboard
stereo calibration — do not rely on the nominal lens focal length for
triangulation accuracy. The default (`≈500 px` for a nominal 3 mm lens at a 640-wide
full-sensor mode) is only a placeholder; replace it with the calibrated value
for your specific rig. `focal_length_px` affects depth (`z = f·B/disparity`),
not the FOV-based coverage validation (which uses the physical lens). See
[`CALIBRATION.md`](CALIBRATION.md) for the chessboard capture and OpenCV
`stereoCalibrate` procedure that produces this value.

### 2.3 Default capture mode

`640×400@120` is the default capture mode. This is a **validated OV9281 binned
mode** (2×2 binning of the native 1280×800, full FOV, half resolution). The
`StereoFrame` buffers are dynamically sized (`std::vector<uint8_t>`) to match
the configured `frame_width × frame_height`, so any supported OV9281 mode works
without code changes.

**Supported high-rate modes** (set `frame_width`, `frame_height`, `target_fps`):

| Mode | Max FPS | Bandwidth (×2 cams) | Use case |
|------|---------|---------------------|----------|
| 640×400 | 210 | ~108 MB/s @210 | default; best balance |
| 640×360 | 210 | ~97 MB/s @210 | 16:9 crop; slightly less vertical |
| 1280×720 | 120 | ~221 MB/s @120 | full resolution; heavier CPU |

> **640×480 is NOT a supported OV9281 mode.** The V4L2 driver will reject it or
> silently remap. Always use 640×400 or 640×360.

**FPS guidance:** higher FPS reduces tracking latency (4.8 ms at 210 FPS vs
8.3 ms at 120 FPS). USB 3.0 bandwidth (~400 MB/s usable) and RPi 5 CPU are not
bottlenecks even at 210 FPS — the detector's 256K-pixel scan is <1 % of NEON
throughput. `target_fps` is a performance knob only: the watchdog timeout is an
**absolute** duration (default 25 ms, deliberately *not* derived from the frame
rate — see AGENTS.md §4.4), and the control loop runs at a fixed 5 ms period.
The startup validator rejects any config whose frame period (`1000/target_fps`)
does not fit inside the watchdog timeout, since the heartbeat advances once per
processed frame. The shipped `config/system_config.yaml` selects the validated
210 FPS mode; the conservative built-in default is 120 FPS.

### 2.4 Image controls (dark-field detection)

The mosquito detector looks for a bright spot on a dark background, so exposure
is locked low by default (`exposure_auto = manual`, `exposure_absolute_us` low,
`gain` low). These are applied via `VIDIOC_S_CTRL` at camera open and are
tunable in `camera_controls`.

---

## 3. Laser

| Parameter | Value |
|-----------|-------|
| Optical power | 2.5 W |
| Class | **Class 4** — instantaneous irreversible eye/skin injury and fire hazard |
| Wavelength | 450 nm (blue) |
| Module dimensions | 33 × 70 mm |
| Drive mode | External “ACC constant-current” drive (seller wording) |
| Cooling | Forced air; required whenever the module is powered |
| Operating voltage | 12 VDC |
| Control | 3-pin interface advertised for TTL switching and PWM power control; pinout, active polarity, and thresholds not yet verified |
| Housing | Anodized aluminium |
| Collimator | Coated optical glass |
| Driver supply | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W |

The same seller listing includes 3.5 W, 5.5 W, 10 W, and 15 W variants. Those
are not installed hardware and must not be used when calculating or describing
this system. The project is specified only for the reported **2.5 W, 33 × 70 mm**
module.

The seller's “TTL/PWM” wording does not establish an electrical interface. The
driver pin order, logic thresholds, active polarity, input current, and
power-up default must be obtained or measured before connection. This project
uses one sustained TTL level per pulse; PWM firing is prohibited because edges
can retrigger the 74HC123 and defeat the hardware duration cap.

### 3.1 Alignment/test laser

The on-hand alignment module is reported only as **5 mW, 12 mm**. Its wavelength,
supply voltage/current, pinout, modulation interface, and labelled class have
not been recorded. It is not automatically a drop-in electrical substitute for
the working laser. Use it for the gated alignment procedure only after those
details are verified and only if the real arm/E-stop/TTL chain still controls
emission. Otherwise obtain a compatible low-power, TTL-controlled visible
alignment module.

**Safety enforcement (in code, not convention):**

| Guard | Mechanism | Location |
|-------|-----------|----------|
| Max pulse | per-cycle duration check + `Laser::enforce_max_pulse`; real software bound ≈ 100 ms config limit + one fixed 5 ms control cycle + jitter (~105 ms — a flat "≤ 100 ms" is a claim the software cannot make, AGENTS.md §4.1) | `FiringController`, `Laser` |
| Pulse backstop (hardware) | 74HC123 one-shot + 74HC08 AND on the TTL line force-cut a stuck-HIGH GPIO 18 at the measured one-shot period (nominally ≈220 ms: the on-hand Hitachi HD74HC123P specifies `t_W = R·C` with no coefficient) with no software or operator involvement — see §3.2 below | wiring, §11a of `HARDWARE_WIRING.md` |
| Cooldown | `cooldown_until_` gates `may_fire()` (configured 10 s; validator floor 1 s) | `FiringController` |
| Motion blanking | no galvo writes while pulse active; settle required before fire | `FiringController` |
| Arm switch | `set_armed` + fire path reject when disarmed; GPIO fault → disarmed | `FiringController`, `ArmSwitch` |
| Watchdog | absolute 25 ms heartbeat timeout (independent of `target_fps`) + bounded startup grace → `emergency_shutdown()` + `SAFE_HALT` | `Watchdog` |
| Coordinate bounds | safe box + galvo cone + voltage-scale DAC (reject, no clamp) | `CoordinateMapper`, `BoundingBox3D` |
| E-stop | active-low mushroom → `SAFE_HALT`; GPIO fault → pressed | `EStop` |
| Config validation | critical engagement mismatches abort startup | `validate_engagement_volume` |
| RAII shutdown | laser GPIO forced LOW on init, on error, and on destruction | `Laser`, `~GpioImpl` |

### 3.2 Hardware pulse-duration backstop (74HC123)

Every *software* mechanism that can end a pulse runs on the control thread; if
that thread stalls with GPIO 18 HIGH, none of them fires. The backstop is a
74HC123 DIP-16 retriggerable monostable plus an SN74HC08N AND gate between the
dedicated qualified GPIO interface and the laser driver: laser TTL =
`translated GPIO18 ∧ one-shot Q`, one-shot
triggered by GPIO 18's rising edge. A normal short pulse passes through
unchanged; a stuck-HIGH GPIO 18 is force-cut when Q times out — with no
software path and no operator action. The fitted timing parts are reported as a
1/2 W carbon-film 220 kΩ resistor and a 50 V monolithic-ceramic 1 µF capacitor.
The on-hand device is a Hitachi HD74HC123P, whose datasheet gives the pulse
equation as `t_W = R_ext · C_ext` with no coefficient, so the fitted parts
compute to `220 kΩ · 1 µF ≈ 220 ms` — not the 99 ms that TI's 0.45 coefficient
would give for a TI part. The full 74HC123 ordering code,
R/C tolerances, and the ceramic capacitor's effective capacitance are not yet
recorded; coefficient and tolerance are vendor-dependent, so measure the actual
period on the assembled circuit. The guarantee holds only if the AND gating is
present, firing stays a single sustained level
(never a PWM burst), and the measured period is what you intend — the
verification procedure is in `PRE_FLIGHT_CHECKLIST.md` §2 and the wiring and
measurement method in `HARDWARE_WIRING.md` §11a. The '123 and the AND gate are single components:
their failure belongs in any FMEA, with the arm switch (cuts 12 V) and E-stop
(cuts mains) as the outer, operator-driven layers.

---

## 4. Engagement envelope (derived)

The minimum/maximum engagement distance is not a single number — it is the
binding result of several constraints. Let `B = 0.12 m` (baseline),
`f = focal_length_px`, `θ_g = 15°` (galvo half-cone), `W = max(|x|,|y|)` lateral
half-width, and `D = √(x²+y²)` lateral corner radius.

| Constraint | Formula | At defaults |
|------------|---------|-------------|
| Galvo cone (corner) | `atan2(D, z) ≤ θ_g` ⇒ `z ≥ D / tan(15°) = 3.73·D` | `D=0.127` (±0.09) → z ≥ 0.47 m |
| Galvo voltage | `θ_g · 0.33 ≤ 5 V` ⇒ `θ_g ≤ 15.15°` | satisfied at ±15° |
| Stereo matchable | `z ≥ f·B / d_max` (d_max ≈ 300 px) | f=500 → z ≥ 0.20 m |
| Detection upper bound (5 mm target, blob area ≥ `min_blob_area_px`) | `(π/4)·(f·s/z)² ≥ A_min` ⇒ `z ≤ f·s·√(π/(4·A_min))` | f=500, s=5 mm, A_min=4 px² → z ≤ 1.11 m |
| Safety/hazard floor | operator | 0.5 m |

**Configured envelope:** `x,y ∈ [−0.09, 0.09] m`, `z ∈ [0.5, 1.0] m`.

The detection bound is the criterion the software actually enforces: the
startup validator cross-checks `min_blob_area_px` against the area a
`target_size_m` object projects to at `z_max` (a ~2.5 px-diameter, ~4.9 px²
blob at z = 1.0 m with the shipped values, clearing the 4 px² floor). The
stricter "≥ 3 px across" rule of thumb quoted in earlier revisions of this
table gives z ≤ 0.83 m and was never what the code checked; detection at the
far end of the envelope is correspondingly marginal — see the small-target
row of `PRE_FLIGHT_CHECKLIST.md` §6.

**Stereo temporal skew (residual):** the two cameras free-run without hardware
sync and are grabbed sequentially, so a pair can be up to one frame period
apart. For a laterally moving target the skew biases disparity and therefore
z by `Δz ≈ z·v·Δt/B` — ≈ 4 cm at z = 1 m, v = 1 m/s, Δt = 4.8 ms, B = 0.12 m.
The runtime records per-camera driver timestamps in each `StereoFrame` and
logs a skew watermark every 512 frames; the z margins of the bounding box must
absorb the residual (AGENTS.md §4.12).

The bounding box is an axis-aligned cuboid, while the reachable volume is a cone
(frustum). To keep the cuboid fully inside the cone, the near-face corner radius
`D = 0.09·√2 = 0.127 m` must satisfy `0.127 / tan(15°) = 0.47 m ≤ z_min`. The
default `z_min = 0.5 m` clears this with ~0.03 m margin, so the validator emits
no warnings. Widening the lateral box requires raising `z_min` proportionally;
the startup validator will warn on any corner that violates the cone.
