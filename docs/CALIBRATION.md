# Calibration Procedure — Mosquito Laser Killer

**WARNING:** Every step in this document is performed with the **2.5 W Class 4
laser disconnected**. Intrinsic and stereo calibration use the cameras only. The
extrinsic camera↔galvo step (§4) uses a **low-power Class 2 / 3R laser** (≤5 mW,
red or green), never the Class 4 module. Wear laser safety eyewear and keep the
E-stop within reach. See [`PRE_FLIGHT_CHECKLIST.md`](PRE_FLIGHT_CHECKLIST.md) for
the surrounding go/no-go workflow — this document is the "how" behind that
checklist's §3 (calibration) and §5 (low-power laser) sections.

---

## 0. What Gets Calibrated and Where It Lands

The default values in `config/system_config.yaml` are **placeholders**. The
system will not aim correctly until they are replaced with measured values for
*your* rig. Three distinct things must be calibrated:

| # | Calibration | Method | Config fields it produces |
|---|-------------|--------|---------------------------|
| 1 | **Per-camera intrinsics** | Chessboard, one camera at a time | (intermediate — feeds stereo) |
| 2 | **Stereo pair** | Chessboard, both cameras together | `stereo.focal_length_px`, `stereo.cx`, `stereo.cy`, `stereo.baseline_m` |
| 3 | **Camera↔galvo extrinsic** | Low-power laser, manual | axis alignment is corrected **mechanically** (shim/rotate the mount); only the *scale* is a config field, `galvo_driver.input_scale_v_per_deg` |

Why each matters for safety, not just accuracy:

- `focal_length_px` sets depth: `z = f·B/disparity`. The aim *angle* is
  `atan2(x, z)` with `x = (u_left − cx)·z/f`, so **z cancels out of the aim** —
  its sole job is the safety discriminator (mosquito at 0.7 m vs. a face across
  the room). A wrong `focal_length_px` corrupts that discriminator. See
  [`HARDWARE_PARAMETERS.md`](HARDWARE_PARAMETERS.md) §2.2.
- `cx` does enter the aim directly (`u_left − cx`). A wrong principal point
  offsets every shot laterally.
- `baseline_m` scales depth linearly; measure the optical-center distance, don't
  eyeball the housing gap.

The startup validator (`validate_engagement_volume()`) rejects a principal point
outside the frame, a non-positive baseline or focal length, and warns if the
camera FOV is narrower than the galvo cone — but it **cannot** tell you your
numbers are *wrong*, only that they are structurally impossible. Calibration is
what makes them right.

---

## 1. Prerequisites

- [ ] Cameras **rigidly** mounted at the final baseline (`stereo.baseline_m`,
      default 0.12 m). Nothing may move between calibration and operation — a
      bumped camera invalidates the whole calibration.
- [ ] Cameras aimed parallel (no toe-in) and aligned vertically. **Mechanical
      parallelism is load-bearing — nothing rectifies at runtime.** The pipeline
      matches raw pixels; there is no `stereoRectify`/`undistort`/`remap` in the
      code path. The only thing absorbing a residual camera-pair misalignment is
      the fixed `detection.epipolar_tolerance_px` (2.0 px), and a residual
      rotation does not just add a constant vertical offset — it makes the
      epipolar error and the effective `cx`/`fx` vary across the frame, biasing
      aim toward the edges. Before trusting `epipolar_tolerance_px` to cover it,
      verify a **static** chessboard produces a near-zero vertical offset between
      the same corner in the two views **across the whole frame** (corners
      included), not just at centre. If it doesn't, fix the mounting, not the
      tolerance.
- [ ] Stable device paths identified and set in `config/system_config.yaml`:
      ```
      ls -l /dev/v4l/by-path/
      ```
      Copy the `by-path` symlinks (not `/dev/videoN`) into `left_camera_device`
      and `right_camera_device`. Left/right must match physical mounting — a
      swap corrupts disparity and aims at the wrong 3D point.
- [ ] A **chessboard** target printed flat and mounted on a rigid backing
      (foamboard/aluminium). Note the **inner-corner** count and **square size
      in metres**. A 9×6 inner-corner board with 25 mm squares is a good default.
      Measure a printed square with calipers — printer scaling is real.
- [ ] Python with OpenCV: `pip install opencv-python numpy`.
- [ ] Capture resolution matches operation. Calibrate at the **same
      `frame_width × frame_height`** you will run (default 640×400). Intrinsics
      are resolution-specific; `cx`, `cy`, `focal_length_px` all scale with it.

---

## 2. Capture Calibration Image Pairs

Capture left/right pairs of the chessboard at many poses. The grabber below
reads the two cameras sequentially (one `read()` then the other), so the frames
are offset by one read latency — **not** hardware-synchronized. That is fine
*only because §2 mandates a static board*: with nothing moving, the offset
captures the same scene. Do not reuse this grabber for anything that moves.

- [ ] 20–30 pairs minimum. More poses beat more images of the same pose.
- [ ] Cover the whole frame: corners, edges, centre. Under-sampled image regions
      calibrate poorly, and the target lives across the whole frame.
- [ ] Vary depth across your engagement volume (0.5–1.0 m) and tilt the board
      ±30° in pitch and yaw. Tilt is what constrains focal length.
- [ ] The **entire** board must be visible in **both** cameras in every kept pair.
- [ ] Keep the board still for each shot (global-shutter OV9281 helps, but motion
      blur still hurts corner localization).

Save pairs as `left/000.png … left/029.png` and `right/000.png …`, index-matched.
A minimal grabber using the configured devices:

```python
import cv2, os
os.makedirs("left", exist_ok=True); os.makedirs("right", exist_ok=True)
capL = cv2.VideoCapture("/dev/v4l/by-path/<your-left-by-path>")
capR = cv2.VideoCapture("/dev/v4l/by-path/<your-right-by-path>")
for cap in (capL, capR):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
n = 0
while True:
    okL, fL = capL.read(); okR, fR = capR.read()
    if not (okL and okR):
        continue
    cv2.imshow("L", fL); cv2.imshow("R", fR)
    k = cv2.waitKey(1) & 0xFF
    if k == ord(" "):                       # SPACE = capture a pair
        cv2.imwrite(f"left/{n:03d}.png", fL)
        cv2.imwrite(f"right/{n:03d}.png", fR)
        print("captured", n); n += 1
    elif k == 27:                           # ESC = done
        break
capL.release(); capR.release(); cv2.destroyAllWindows()
```

---

## 3. Run Intrinsic + Stereo Calibration

The OV9281 is monochrome; calibrate in grayscale. Set `BOARD` and `SQUARE_M` to
**your** board.

```python
import cv2, glob, numpy as np

BOARD    = (9, 6)     # inner corners (cols, rows) — NOT squares
SQUARE_M = 0.025      # square edge length in METERS (measure it)
IMG_SIZE = (640, 400) # must equal frame_width x frame_height in the config

# 3D object points for one board pose (z=0 plane), scaled to metres.
objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_M

objpoints, imgL, imgR = [], [], []
crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

for fl, fr in zip(sorted(glob.glob("left/*.png")), sorted(glob.glob("right/*.png"))):
    gL = cv2.imread(fl, cv2.IMREAD_GRAYSCALE)
    gR = cv2.imread(fr, cv2.IMREAD_GRAYSCALE)
    okL, cL = cv2.findChessboardCorners(gL, BOARD, None)
    okR, cR = cv2.findChessboardCorners(gR, BOARD, None)
    if not (okL and okR):
        print("skip (board not found):", fl); continue
    cL = cv2.cornerSubPix(gL, cL, (11, 11), (-1, -1), crit)
    cR = cv2.cornerSubPix(gR, cR, (11, 11), (-1, -1), crit)
    objpoints.append(objp); imgL.append(cL); imgR.append(cR)

print(f"usable pairs: {len(objpoints)}")

# 1) Per-camera intrinsics
rmsL, KL, dL, *_ = cv2.calibrateCamera(objpoints, imgL, IMG_SIZE, None, None)
rmsR, KR, dR, *_ = cv2.calibrateCamera(objpoints, imgR, IMG_SIZE, None, None)
print(f"intrinsic RMS  L={rmsL:.3f}  R={rmsR:.3f}  (target < 0.5 px)")

# 2) Stereo — fix intrinsics, solve the rig geometry
rms, KL, dL, KR, dR, R, T, *_ = cv2.stereoCalibrate(
    objpoints, imgL, imgR, KL, dL, KR, dR, IMG_SIZE,
    flags=cv2.CALIB_FIX_INTRINSIC, criteria=crit)
print(f"stereo RMS = {rms:.3f} px   (target < 1.0 px)")

fx, fy = KL[0, 0], KL[1, 1]
cx, cy = KL[0, 2], KL[1, 2]
baseline_m = float(np.linalg.norm(T))     # |T| = optical-center distance

print("\n--- paste into config/system_config.yaml stereo: ---")
print(f"  baseline_m: {baseline_m:.5f}")
print(f"  focal_length_px: {fx:.2f}   # (fy={fy:.2f}; expect fx≈fy)")
print(f"  cx: {cx:.2f}")
print(f"  cy: {cy:.2f}")

# 3) Distortion CHECK — the runtime never undistorts, so these must be small.
#    k1,k2 are the dominant radial terms. |k1| >~ 0.05 means real barrel/pincushion.
print(f"\ndistortion  L k1={dL[0,0]:+.4f} k2={dL[0,1]:+.4f}   "
      f"R k1={dR[0,0]:+.4f} k2={dR[0,1]:+.4f}   (want |k1|,|k2| < ~0.05)")
```

### Acceptance thresholds

| Metric | Target | If it fails |
|--------|--------|-------------|
| Per-camera intrinsic RMS | **< 0.5 px** | More/better poses; check board size + square measurement |
| Stereo RMS | **< 1.0 px** | Re-shoot; ensure full board in both views, less blur |
| `fx` vs `fy` | within ~1–2% | Large gap ⇒ wrong square size or bad corners |
| `baseline_m` vs measured | within a few mm | Sanity-check against a physical tape measure |
| `cx`, `cy` | near frame centre (320, 200) | Far off ⇒ suspect a bad pose set; validator rejects outside-frame |
| Distortion `k1`, `k2` | **`\|k1\|`,`\|k2\|` < ~0.05** | Lens is not "distortion-free" — see below |

Do not ship a calibration that misses these — a high RMS means the depth
discriminator is unreliable, and depth is the guard that separates a mosquito
from a bystander.

### The distortion coefficients are checked, not applied

The script computes `dL`/`dR` but the **runtime never undistorts** — there is no
`undistort`/`initUndistortRectifyMap` in the pipeline, and the config calls the
3 mm lens "distortion-free." That is an **assumption you must verify, not a
given.** The distortion print above is that check: if `|k1|` or `|k2|` exceeds
~0.05, the lens has real barrel/pincushion, and because nothing corrects it, the
effective principal point and focal length drift toward the ±15° edges — exactly
where §5's scale check lives, so the error masquerades as a galvo-scale problem
you will never tune out.

Two quick confirmations:

- **Numeric:** `|k1|`, `|k2|` both under ~0.05 (printed by the script).
- **Visual:** overlay a straight edge on a raw frame corner (or view the
  chessboard at a frame corner) — the board's outer rows/columns must stay
  straight, not bow. Bowing at the corners = distortion the runtime won't remove.

If the lens fails this, the options are a lower-distortion lens or adding an
undistort stage to the capture path (a code change, out of scope for this doc).
Do not paste the numbers and proceed as if the lens were ideal.

---

## 4. Transfer to Config and Validate

- [ ] Paste the four values into the `stereo:` block of
      `config/system_config.yaml`.
- [ ] `cx ≈ frame_width / 2` and `cy ≈ frame_height / 2` — small offsets are
      expected and correct; large ones mean a calibration problem.
- [ ] Start the application and confirm startup validation passes — **no**
      `[CONFIG] Aborting` line. A non-positive focal length/baseline or a
      principal point outside the frame aborts here (§4.10 of `AGENTS.md`).
- [ ] Follow [`PRE_FLIGHT_CHECKLIST.md`](PRE_FLIGHT_CHECKLIST.md) §4 (Detection
      Validation): move a target at known distances (0.5, 0.75, 1.0 m) and
      confirm the projected position and reported `z` are correct. A consistent
      depth error here points straight back to `focal_length_px` or `baseline_m`.

---

## 5. Camera↔Galvo Extrinsic Alignment (Low-Power Laser)

Stereo calibration puts the target in the **camera** coordinate frame. The galvo
lives in its **own** frame, and the code currently assumes the two frames are
aligned (see `PRE_FLIGHT_CHECKLIST.md` §6, "No camera/galvo extrinsic
calibration"). This step measures and removes that residual offset manually. It
is the practical calibration behind checklist §5.

**Use a low-power Class 2 / 3R laser (≤5 mW), mounted exactly where the Class 4
module will sit, with the same arm/E-stop/TTL wiring.** The Class 4 laser stays
disconnected.

- [ ] Confirm the low-power laser still needs the arm switch ON **and** GPIO 18
      HIGH to emit; with the arm switch OFF, no beam.
- [ ] Place a target at the **centre** of the bounding box, arm, and let a track
      confirm (~3 consecutive motion frames).
- [ ] Observe where the beam lands relative to the target.

Correct in this order — cheapest cause first:

1. **Left/right swap.** If the beam is wildly off or mirrored, the cameras are
   swapped. Re-check the `by-path` assignment.
2. **Lateral / vertical constant offset.** A consistent miss in one direction is
   a physical mount misalignment between the camera optical axis and the galvo
   neutral axis. Mechanically shim/rotate the galvo (or laser mount) until a
   centre-box target is hit. This is the extrinsic offset the software does not
   model — remove it physically.
3. **Scale error (miss grows toward the edges).** If centre is dead-on but the
   beam under/overshoots as the target moves toward the ±15° edges, the galvo
   command scale is off: adjust `galvo_driver.input_scale_v_per_deg` (default
   0.33 V/deg). Larger value ⇒ more deflection per commanded degree. Re-validate
   that `galvo_limits` still fit the `dac_max_diff_voltage` budget after any
   change (the startup validator enforces this).
4. **Depth-dependent miss.** If the beam is right at 0.5 m but wrong at 1.0 m
   (or vice-versa), the error is in *depth*, i.e. back to §3 (`focal_length_px`,
   `baseline_m`), not the galvo.

- [ ] Sweep the target slowly across the full box (within the ±15° cone) and
      confirm the beam tracks at every position and depth.
- [ ] Disarm — beam must stop immediately. Press E-stop — beam off and system
      halts. Only after all of §5 passes does the go/no-go gate for the Class 4
      laser open (`PRE_FLIGHT_CHECKLIST.md` §7).

---

## 6. When to Re-Calibrate

Calibration is only valid while the rig is unchanged. Re-run the relevant step
after any of:

- A camera is bumped, remounted, refocused, or the baseline changes → **§2–4**
  (stereo) and then **§5** (extrinsic).
- The lens is swapped → **§2–4** and **§5**.
- The capture resolution (`frame_width`/`frame_height`) changes → **§2–4**
  (intrinsics are resolution-specific).
- The galvo, laser mount, or their alignment is disturbed → **§5**.
- Consistent aiming error observed in operation → diagnose with the §5 decision
  order (swap → offset → scale → depth).

There is no on-disk persistence and no online recalibration (`AGENTS.md` §10):
calibration lives entirely in `config/system_config.yaml`, so keep that file
under version control and note the calibration date/RMS in your commit message.
