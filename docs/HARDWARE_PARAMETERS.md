# Hardware Parameters — Mosquito Laser Killer

This document records the measured/quoted parameters of the galvanometer, the
cameras, and the laser, and derives the **engagement envelope** that the software
enforces. The runtime configuration lives in `config/system_config.yaml`; a
startup validator (`src/safety/config_validator.cpp`, see
`validate_engagement_volume`) checks that the configured safe firing volume,
galvo limits, and camera field of view are mutually consistent and prints
warnings on mismatch. **Critical** mismatches abort startup.

---

## 1. Galvanometer + Driver

### 1.1 Galvo head

| Parameter | Value |
|-----------|-------|
| Maximum scan angle | ±30° optical (default ±15°) |
| Mirror | 11 × 7 × 0.7 mm, dielectric film, >99% reflectivity @ 45° AoI |
| Wavelength coverage | 400–700 nm |
| Operating voltage | ±12 V |
| Operating temperature | 0 °C to +45 °C |
| Storage temperature | −10 °C to +60 °C |
| Operating noise | ≤ 30 dB |
| Average current | 0.5 A |
| Peak current | 1.5 A (1 A) |
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

### 1.3 Electrical chain (DAC → driver → galvo)

```
MCP4922 (12-bit, 0–5 V unipolar per channel)
   └── differential pair:  ChA = V+, ChB = (4095-x) inverted  →  ±5 V swing
        └── Driver input (±5 V, 0.33 V/°)  →  ±15° optical (default)
             └── Galvo mirror  →  beam steered ±15° optical
```

**Key consequence:** the ±5 V differential DAC range, combined with the
**0.33 V/°** driver input scale, commands at most **±5 / 0.33 ≈ ±15.15°** optical.
The galvo head can mechanically reach ±30° optical, but only ±10 V of drive would
get there — unreachable from this DAC. Therefore the software hard-limits the
galvo to **±15°** (`galvo_limits`), and the validator flags any configuration
whose half-cone exceeds `dac_max_diff_voltage / input_scale_v_per_deg`.

At DAC code `c` (0…4095), the differential voltage is
`V_diff = (2·c/4095 − 1) · 5 V`, and the optical angle is
`θ = V_diff / 0.33`. Center `c = 2048` → 0 V → 0°.

---

## 2. Cameras (OV9281, stereo pair)

| Parameter | Value |
|-----------|-------|
| Sensor | OV9281, 1/4", global-shutter monochrome |
| Native array | 1280 × 800, 3 µm pixels |
| Sensor width / height | 3.84 mm × 2.4 mm |
| Interface | USB3 UVC, 120 FPS |
| High-rate modes | MJPG 1280×720@120, 640×400@210, 640×360@210 |
| Low-rate mode | YUV 1280×720@10 |
| Adjustable V4L2 controls | Brightness, Contrast, Saturation, White balance, Gamma, Sharpness, Exposure, Gain |

### 2.1 Lens selection

| Lens | H-FOV (½-FOV) | Distortion | Verdict for ±15° galvo cone |
|------|---------------|-----------|-----------------------------|
| 1.3 mm | ~112° (56°) | yes | too wide; fisheye breaks the pinhole stereo model |
| 2.4 mm | ~77° (39°) | yes | usable but requires an undistortion stage |
| **3 mm** | **~65° (33°)** | **free** | **chosen — covers ±15° cone with margin, no undistortion** |
| 6 mm | ~36° (18°) | free | too narrow for a ±15° cone — camera can't see galvo corners |

Field of view is derived from the physical lens (resolution/binning independent):

```
H-FOV = 2 · atan(sensor_width_mm / (2 · focal_length_mm))
V-FOV = 2 · atan(sensor_height_mm / (2 · focal_length_mm))
```

For the selected 3 mm lens: H-FOV ≈ 65.2°, V-FOV ≈ 43.6°.

### 2.2 Focal length in pixels (calibration)

`stereo.focal_length_px` is a **calibrated** quantity obtained from a chessboard
stereo calibration — do not rely on the nominal lens focal length for
triangulation accuracy. The default (`≈500 px` for the 3 mm lens at a 640-wide
full-sensor mode) is only a placeholder; replace it with the calibrated value
for your specific rig. `focal_length_px` affects depth (`z = f·B/disparity`),
not the FOV-based coverage validation (which uses the physical lens).

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
throughput. The watchdog's heartbeat period auto-derives from `target_fps`.

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
| Focus | Adjustable |
| Control | TTL/PWM |
| Driver supply | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W |

**Safety enforcement (in code, not convention):**

| Guard | Mechanism | Location |
|-------|-----------|----------|
| Max pulse ≤ 100 ms | per-cycle duration check + `Laser::enforce_max_pulse` | `FiringController`, `Laser` |
| Cooldown ≥ 10 s | `cooldown_until_` gates `may_fire()` | `FiringController` |
| Motion blanking | no galvo writes while pulse active; settle required before fire | `FiringController` |
| Arm switch | `set_armed` + fire path reject when disarmed; GPIO fault → disarmed | `FiringController`, `ArmSwitch` |
| Watchdog | 3 missed heartbeats → `emergency_shutdown()` + `SAFE_HALT` | `Watchdog` |
| Coordinate bounds | safe box + galvo cone + voltage-scale DAC (reject, no clamp) | `CoordinateMapper`, `BoundingBox3D` |
| E-stop | active-low mushroom → `SAFE_HALT`; GPIO fault → pressed | `EStop` |
| Config validation | critical engagement mismatches abort startup | `validate_engagement_volume` |
| RAII shutdown | laser GPIO forced LOW on init, on error, and on destruction | `Laser`, `~GpioImpl` |

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
| Detection upper bound (5 mm, ≥3 px) | `z ≤ f·0.005/3` | f=500 → z ≤ 0.83 m |
| Safety/hazard floor | operator | 0.5 m |

**Configured envelope:** `x,y ∈ [−0.09, 0.09] m`, `z ∈ [0.5, 1.0] m`.

The bounding box is an axis-aligned cuboid, while the reachable volume is a cone
(frustum). To keep the cuboid fully inside the cone, the near-face corner radius
`D = 0.09·√2 = 0.127 m` must satisfy `0.127 / tan(15°) = 0.47 m ≤ z_min`. The
default `z_min = 0.5 m` clears this with ~0.03 m margin, so the validator emits
no warnings. Widening the lateral box requires raising `z_min` proportionally;
the startup validator will warn on any corner that violates the cone.
