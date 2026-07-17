# Hardware Wiring Guide — Mosquito Laser Killer

**WARNING:** This system controls a **Class 4 laser** (2.5 W, 450 nm). Class 4 lasers cause instantaneous, irreversible eye injury and can ignite materials. Do not apply power to the laser until all enclosures, beam dumps, interlocks, and OD 4+ safety eyewear are in place. Read `AGENTS.md` before modifying code or wiring.

This document is the single source of truth for wiring the stereoscopic laser-targeting hardware. It matches the source code in `src/hal/mcp4922.cpp`, `src/hal/differential_galvo_driver.cpp`, and `src/control/coordinate_mapper.cpp`.

---

## Table of Contents

1. [System Block Diagram](#1-system-block-diagram)
2. [Bill of Materials](#2-bill-of-materials)
3. [Mains Power & Physical E-Stop](#3-mains-power--physical-e-stop)
4. [DC Power Distribution](#4-dc-power-distribution)
5. [Arming Switch & GPIO 24 Sensing Circuit](#5-arming-switch--gpio-24-sensing-circuit)
6. [E-Stop GPIO Circuit](#6-e-stop-gpio-circuit)
7. [Raspberry Pi 5 GPIO Connections](#7-raspberry-pi-5-gpio-connections)
8. [SPI Bus & MCP4922 DAC Wiring](#8-spi-bus--mcp4922-dac-wiring)
9. [Logic Level Shifter](#9-logic-level-shifter)
10. [Galvo Driver Connections](#10-galvo-driver-connections)
11. [Laser Module Wiring](#11-laser-module-wiring)
12. [Camera Wiring](#12-camera-wiring)
13. [Grounding & Shielding](#13-grounding--shielding)
14. [How the ±5 V Differential Signal Works](#14-how-the-5-v-differential-signal-works)
15. [Power-Up / Power-Down Sequence](#15-power-up--power-down-sequence)
16. [Bench Testing Without the Laser](#16-bench-testing-without-the-laser)

---

## 1. System Block Diagram

```
230V AC L ──[DPST E-STOP NC]──┬──► ±15 VDC supply ──► Galvo driver board
                              │                       (±15 V, ±5 V input)
                              │
                              └──► 12 VDC supply ────┬──► Laser driver module
                                                       │    (power, via ARM switch)
                                                       │
                                                       └──► RPi 5 + ARM switch sense
                                
     LEFT camera ──────►  ┌──────────────┐
                          │              │
     RIGHT camera ─────►  │  Raspberry   │──SPI0──► 2× MCP4922 ──► X/Y galvo drivers
                          │   Pi 5       │
     ARM switch ───────►  │  GPIO24/25   │──GPIO18──► Laser TTL
     E-Stop ───────────►  │              │
                          └──────────────┘
```

---

## 2. Bill of Materials

| Component | Specification | Qty | Purpose |
|-----------|---------------|-----|---------|
| Host | Raspberry Pi 5 (8 GB recommended) | 1 | Real-time control and vision |
| Cameras | 2× OV9281 global-shutter monochrome, USB3 UVC | 2 | Stereo vision |
| Laser | 2.5 W focusable TTL/PWM 450 nm Class 4 blue | 1 | Target neutralization |
| Laser PSU | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W | 1 | Laser driver power |
| Galvo PSU | ±15 VDC (or 15 V + 15 V) supply, ≥2 A | 1 | Galvo driver power |
| X-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Differential X-axis drive |
| Y-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Differential Y-axis drive |
| Level shifter | 4-channel bidirectional 3.3 V ↔ 5 V | 1 | SPI and laser TTL level shifting |
| Monostable | SN74HC123N, DIP-16 dual retriggerable one-shot | 1 | Laser TTL pulse-duration backstop (§11a) |
| AND gate | SN74HC08N, DIP-14 quad 2-input AND | 1 | Gates laser TTL = GPIO18 ∧ one-shot-Q (§11a) |
| Resistor | 220 kΩ, 1/4 W | 1 | 74HC123 timing (R_ext) |
| Capacitor | 1 µF | 1 | 74HC123 timing (C_ext); 0.1 µF for VCC decoupling |
| Arm switch | Lever SPST toggle, rated ≥3 A @ 12 VDC | 1 | Laser power + GPIO arm sense |
| E-Stop | Mushroom DPST push-button, NC contacts | 1 | Physical power kill + GPIO sense |
| Resistor | 10 kΩ, 1/2 W | 3 | Arm switch series, E-stop series (×2) |
| Resistor | 3.3 kΩ, 1/2 W | 2 | Arm switch / E-stop pull-downs |
| Zener diode | BZX55C3V3, 1/2 W | 2 | GPIO input overvoltage clamp |
| Capacitor | 100 nF ceramic | 2 | Arm switch / E-stop debounce |
| Enclosure | Laser-safe interlocked case with beam dump | 1 | Operator protection |
| Safety eyewear | OD 4+ @ 450 nm | 1+ | Mandatory during alignment |

---

## 3. Mains Power & Physical E-Stop

The mushroom E-Stop is the last-line-of-defense physical disconnect. It kills AC power to both DC supplies.

```
Wall L ──► [DPST E-STOP NC] ──► Wago L ──┬──► ±15 V supply AC L
                                           └──► 12 V supply AC L
Wall N ───────────────────────── Wago N ──┬──► ±15 V supply AC N
                                           └──► 12 V supply AC N
PE/GND ──────────────────────── Wago PE ─┬──► ±15 V supply PE
                                           └──► 12 V supply PE
```

### Important

- Use the **NC (Normally Closed)** contact block of the DPST button.
- Wire only the **Live (Phase)** wire through the E-Stop. Neutral and Earth pass straight through.
- **Both poles** of the DPST are used for redundancy:
  - Pole 1 carries the mains Live (AC side).
  - Pole 2 is used for the GPIO E-Stop sense circuit in Section 6.
- Do not rely on software alone. The physical E-Stop must remove all power that can move the beam or fire the laser.

---

## 4. DC Power Distribution

| Supply | Output | Feeds | Notes |
|--------|--------|-------|-------|
| ±15 V (or 15 V dual) | ±15 VDC | Galvo driver board | Provides galvo motor drive |
| 12 V (Mean Well LRS-50-12) | 12 VDC | Laser driver via arm switch + Raspberry Pi 5 via USB-C PD | Keep laser on a switched branch |

The Raspberry Pi 5 itself is powered from its 5 V USB-C supply. The 12 V supply feeds the laser driver module through the arm switch.

---

## 5. Arming Switch & GPIO 24 Sensing Circuit

The arm switch performs **two functions**: it switches 12 V to the laser driver **and** it tells the Pi (via GPIO 24) that the system is armed. The laser cannot fire unless the arm switch is ON and the software also asserts the GPIO 18 TTL signal.

```
12 V supply + ──► [ARM SPST] ──┬──► Laser driver +VIN
                                 │
                                 └──► 10 kΩ ──►┬──► GPIO 24 (RPi)
                                               ├── 3.3 kΩ ──► GND
                                               ├── 100 nF ──► GND
                                               └── BZX55C3V3 cathode ──► GND
```

### Component notes

- **10 kΩ**: current-limiting series resistor from the 12 V arm signal.
- **3.3 kΩ**: lower leg of the voltage divider. GPIO 24 sees:
  `12 V × 3.3 kΩ / (10 kΩ + 3.3 kΩ) ≈ 2.98 V` — a valid 3.3 V logic HIGH.
- **100 nF**: filters switch bounce and high-frequency noise.
- **BZX55C3V3**: cathode to the junction, anode to GND. Clamps transients above ~3.3 V.
- The arm switch must be rated for the laser driver current (typically < 1 A at 12 V).

---

## 6. E-Stop GPIO Circuit

The DPST mushroom button also provides a software-monitored E-Stop on GPIO 25. The control thread polls this pin every cycle; if it goes LOW, the system transitions to `SAFE_HALT`, forces the laser LOW, and centers the galvos.

```
3.3 V (RPi) ──► [E-Stop pole 2 NC] ──► 10 kΩ ──►┬──► GPIO 25 (RPi)
                                                    ├── 3.3 kΩ ──► GND
                                                    ├── 100 nF ──► GND
                                                    └── BZX55C3V3 cathode ──► GND
```

### Operation

| State | Pole 2 contact | GPIO 25 | Software interpretation |
|-------|----------------|---------|-------------------------|
| Released (normal) | Closed | HIGH (3.3 V) | System OK |
| Pressed | Open | LOW (pulled down) | Emergency stop → SAFE_HALT |
| Wire broken | Open | LOW (pulled down) | Emergency stop → SAFE_HALT |

This is fail-safe: a broken wire or pressed button produces the same safe state.

---

## 7. Raspberry Pi 5 GPIO Connections

| Function | GPIO | Pin | Direction | Voltage | Destination |
|----------|------|-----|-----------|---------|-------------|
| Laser TTL | GPIO 18 | 12 | Output | 3.3 V → level shifter → 5 V | Laser driver TTL input |
| Arm switch sense | GPIO 24 | 18 | Input | 2.98 V HIGH | Arm switch voltage divider |
| E-Stop sense | GPIO 25 | 22 | Input | 3.3 V HIGH | E-Stop NC + pull-down |
| SPI0 MOSI | GPIO 10 | 19 | Output | 3.3 V → level shifter → 5 V | Both MCP4922 SDI |
| SPI0 MISO | GPIO 9 | 21 | Input | 3.3 V | Not used by MCP4922 (write-only) |
| SPI0 SCLK | GPIO 11 | 23 | Output | 3.3 V → level shifter → 5 V | Both MCP4922 SCK |
| SPI0 CE0 | GPIO 8 | 24 | Output | 3.3 V | MCP4922 #1 /CS (X-axis) |
| SPI0 CE1 | GPIO 7 | 26 | Output | 3.3 V | MCP4922 #2 /CS (Y-axis) |
| 3.3 V | 3V3 | 1 / 17 | Power | 3.3 V | Pull-up, E-stop sense |
| 5 V | 5V | 2 / 4 | Power | 5 V | Level shifter HV side, MCP4922 VDD |
| GND | GND | 6, 9, 14, 20, 25, 30, 34, 39 | Power | 0 V | Common ground |

---

## 8. SPI Bus & MCP4922 DAC Wiring

Two MCP4922 DACs share the SPI0 bus. The chip-select pins determine which axis receives the command.

```
RPi GPIO 10 (MOSI) ──► level shifter ──► MCP4922 #1 SDI
RPi GPIO 10 (MOSI) ──► level shifter ──► MCP4922 #2 SDI

RPi GPIO 11 (SCLK) ──► level shifter ──► MCP4922 #1 SCK
RPi GPIO 11 (SCLK) ──► level shifter ──► MCP4922 #2 SCK

RPi GPIO 8  (CE0) ──────────────────────► MCP4922 #1 /CS   (X-axis DAC)
RPi GPIO 7  (CE1) ──────────────────────► MCP4922 #2 /CS   (Y-axis DAC)

RPi 5 V ────────────────────────────────► MCP4922 #1 VDD
RPi 5 V ────────────────────────────────► MCP4922 #2 VDD

5 V reference ──────────────────────────► MCP4922 #1 Vref
5 V reference ──────────────────────────► MCP4922 #2 Vref

GND (common) ───────────────────────────► MCP4922 #1 AGND/VSS
GND (common) ───────────────────────────► MCP4922 #2 AGND/VSS
```

### MCP4922 channel assignment

| DAC | Chip select | Channel A | Channel B | Axis |
|-----|-------------|-----------|-----------|------|
| X-axis | GPIO 8 / CE0 | X+ | X- (inverted) | Horizontal mirror |
| Y-axis | GPIO 7 / CE1 | Y+ | Y- (inverted) | Vertical mirror |

Both DACs are powered from the **same 5 V rail** as the level shifter high side. This guarantees that a full-scale code (`4095`) outputs `5.0 V` and mid-scale (`2048`) outputs `2.5 V` on each channel.

---

## 9. Logic Level Shifter

The Raspberry Pi GPIO outputs 3.3 V logic. The MCP4922, when powered from 5 V, requires a logic HIGH of at least `0.7 × VDD = 3.5 V`. A 3.3 V signal may not be reliably recognized, so shift it to 5 V. The laser TTL input also commonly expects 5 V.

```
RPi 3.3 V ──► LV
RPi GND  ───► LGND
RPi MOSI ───► A1 ──► B1 ──► 5 V MOSI to DACs
RPi SCLK ───► A2 ──► B2 ──► 5 V SCLK to DACs
RPi GPIO18 ─► A3 ──► B3 ──► 5 V laser TTL
     (spare) ─► A4 ──► B4

RPi 5 V ────► HV
RPi GND  ───► HGND (common with LGND and all GNDs)
```

### Wiring check

- LV = 3.3 V, HV = 5 V.
- Low-voltage ground and high-voltage ground are tied together at one point.
- Both grounds are tied to the Raspberry Pi ground, the DAC ground, and the laser driver ground.

---

## 10. Galvo Driver Connections

The galvo driver board accepts a **differential analog input** for each axis. The two MCP4922 channels per axis drive the positive and negative inputs.

```
MCP4922 #1 ChA (X+) ──► X driver IN+
MCP4922 #1 ChB (X-) ──► X driver IN-

MCP4922 #2 ChA (Y+) ──► Y driver IN+
MCP4922 #2 ChB (Y-) ──► Y driver IN-

DAC/RPi GND ────────────► X driver GND
DAC/RPi GND ────────────► Y driver GND
```

### Signal mapping

| DAC code | ChA voltage | ChB voltage | Differential (ChA − ChB) | Optical angle (at 0.33 V/°) |
|----------|-------------|-------------|----------------------------|------------------------------|
| 0 | 0.0 V | 5.0 V | −5.0 V | −15.15° |
| 2048 | 2.5 V | 2.5 V | 0.0 V | 0° |
| 4095 | 5.0 V | 0.0 V | +5.0 V | +15.15° |

Do **not** ground `IN-`. The system is designed for differential drive; grounding the negative input would halve the usable range and invalidate the coordinate mapping.

---

## 11. Laser Module Wiring

The laser module has two electrical connections: a **12 V power input** and a **TTL input**.

```
12 V supply + ──► [ARM SPST switch] ──► Laser driver +VIN
12 V supply − ────────────────────────► Laser driver GND

Level shifter B3 ──► [74HC123 + AND gate backstop, §11a] ──► Laser driver TTL
Laser driver GND ────────────────────► Common GND
```

### Safety notes

- The laser driver receives **no power** until the arm switch is ON. This is the hardware interlock.
- The TTL input is the **software trigger**, but it passes through the pulse-duration backstop of §11a — the laser fires only when GPIO 18 is HIGH, the arm switch is ON, the E-Stop is released, the one-shot has not timed out, and all software safety gates are satisfied.
- Keep the TTL line short and away from the laser power cable to reduce noise.
- The laser module chassis must be bonded to protective earth.

---

## 11a. Laser TTL Pulse-Duration Backstop (74HC123)

Every *software* mechanism that ends a laser pulse (max-pulse enforcement, watchdog, E-stop poll) runs on the **control thread**. If that thread hangs with GPIO 18 stuck HIGH, no software turns the laser off. This one-shot is the **independent hardware enforcer** of the ≤100 ms pulse bound — it is not on the control thread and needs neither software nor an operator to act. See AGENTS.md §4.1.

### Circuit

The laser TTL is gated by `GPIO18 (5 V) AND one-shot-Q`. The one-shot is triggered by GPIO 18's rising edge; when it times out, the AND gate force-drives the laser TTL LOW even if GPIO 18 is still HIGH.

```
Level shifter B3 (GPIO18 @ 5 V) ─┬─────────────────────► 74HC08 AND  in A ─┐
                                 │                                          ├─► Laser driver TTL
                                 └─► 74HC123  1B (pin 2, +edge trigger)     │
                                     74HC123  1Q (pin 13) ──────────────────┘ (AND in B)
                                     74HC123  1CLR (pin 3) ◄── GPIO18 @ 5 V  (re-arm between shots)

Timing network (channel 1):
    74HC123  1Cext (pin 14) ──┬── 1 µF ──┬── 1Rext/Cext (pin 15)
    74HC123  1Rext/Cext (pin 15) ── 220 kΩ ──► +5 V
    74HC123  VCC (pin 16) ──► +5 V ;  GND (pin 8) ──► common GND ;  0.1 µF VCC↔GND decoupling
    Unused channel 2 (pins 9,10,11): tie 2A/2B/2CLR to defined levels; leave 2Cext/2Rext open
```

### Timing

`t_W ≈ 0.45 · R_ext · C_ext = 0.45 · 220 kΩ · 1 µF ≈ 99 ms`

`K = 0.45` is nominal for the 74HC123 at V_CC = 5 V, C_ext > 1 nF; R and (electrolytic) C tolerance can move t_W ±20 %. **Measure it on a scope.** ~99 ms sits just below the ~105 ms real software bound, so the hardware becomes the binding limit and will also cap legitimate max-length pulses. To make the backstop trip only on a *failure* instead, raise the period above the software bound (e.g. C_ext = 1.5 µF → ~130–150 ms).

### Verification (mandatory before connecting the Class 4 laser)

- [ ] **Short pulse passes through:** drive GPIO 18 HIGH for ~10 ms; the laser TTL must be HIGH for ~10 ms (not stretched to ~99 ms). If it stretches, Q is driving the TTL *alone* — the AND gate is missing or miswired. Fix before proceeding.
- [ ] **Stuck-HIGH is capped:** hold GPIO 18 HIGH indefinitely; the laser TTL must go LOW at the measured t_W and stay LOW.
- [ ] **Period measured** on a scope and recorded; matches intent (§ Timing above).
- [ ] **No PWM firing:** confirm the firing path drives a single sustained level per pulse — a retriggering burst within t_W would hold the output HIGH and defeat the cap.

---

## 12. Camera Wiring

The two OV9281 cameras connect to the Raspberry Pi 5 USB 3.0 ports. Use **USB 3.0** ports (blue) for full bandwidth; USB 2.0 ports will not sustain 640×400@120 reliably.

### Camera identification

UVC camera enumeration order can change after reboot. Identify cameras by their stable USB port path, not by `/dev/videoN`.

```bash
ls -l /dev/v4l/by-path/
```

Copy the full by-path symlinks into `config/system_config.yaml`:

```yaml
left_camera_device: "/dev/v4l/by-path/platform-1f00100000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"
right_camera_device: "/dev/v4l/by-path/platform-1f00100000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0"
```

Physically label each camera LEFT/RIGHT after determining its by-path. Swapping them corrupts stereo disparity and would aim the laser at wrong 3D positions.

---

## 13. Grounding & Shielding

A single-point ground is critical for low noise and safety.

- Tie the Raspberry Pi GND, DAC AGND, level shifter GND, laser driver GND, galvo driver GND, and all 0 V returns to a common ground plane or star ground.
- Use shielded USB 3 cables for the cameras and route them away from the laser power cable.
- The laser module chassis and metal enclosures must be bonded to protective earth (PE), not just logic GND.
- Avoid ground loops. If you use multiple DC supplies, connect their 0 V outputs at one central point.

---

## 14. How the ±5 V Differential Signal Works

A Raspberry Pi cannot output a negative voltage. The system creates a **negative-going differential signal** by driving two positive DAC outputs in opposite directions.

For one axis:

- Channel A outputs `V+` in the range 0–5 V.
- Channel B outputs `V−` in the range 0–5 V, always inverted relative to Channel A.
- The galvo driver measures the difference: `V_diff = V+ − V−`.

Because the two channels are complementary:

- `V+ = 5 V`, `V− = 0 V` → `V_diff = +5 V`
- `V+ = 0 V`, `V− = 5 V` → `V_diff = −5 V`
- `V+ = 2.5 V`, `V− = 2.5 V` → `V_diff = 0 V`

With the driver input scale of **0.33 V/°**, this ±5 V differential range commands approximately **±15.15° optical**. The software limits commands to **±15°** (`galvo_limits` in `config/system_config.yaml`).

The code computes the DAC code in `CoordinateMapper::angle_to_dac_code`:

```
V_diff = angle_deg × 0.33
normalized = (V_diff / Vref) + 1.0          # Vref = 5.0 V
code = normalized × (4095 / 2)
```

`DifferentialGalvoDriver::set_position` then writes the code to `ChA` and the complement `4095 − code` to `ChB`.

---

## 15. Power-Up / Power-Down Sequence

### Power-up

1. Verify the E-Stop is released.
2. Verify the arm switch is **OFF**.
3. Apply 230 V AC to both power supplies.
4. Wait for the Raspberry Pi 5 to boot.
5. Verify camera streams start and no errors are logged.
6. Put on safety eyewear and confirm the enclosure is closed.
7. Turn the arm switch **ON** only when ready to run.

### Power-down

1. Turn the arm switch **OFF**.
2. Wait 10 seconds for the laser cooldown to complete.
3. Trigger software shutdown (`sudo shutdown now`).
4. After the Pi halts, remove 230 V AC.

### Emergency

Press the mushroom E-Stop at any time. This removes AC power from both supplies and forces the GPIO 25 E-Stop pin LOW. The system enters `SAFE_HALT` and cannot resume until power is cycled.

---

## 16. Bench Testing Without the Laser

For initial bring-up, leave the laser **disconnected or switched off** and use a multimeter or oscilloscope on the DAC outputs.

1. Build the Pi, DACs, level shifter, and galvo drivers.
2. Connect the galvo drivers **only** — do not connect the galvo motors yet if you want to test the electronics without mechanical motion.
3. Power the Pi. Run `mosquito_laser_killer` with the arm switch OFF.
4. Verify the DAC outputs are near 2.5 V on all four channels at startup.
5. Apply a test target (or use a test mode) and verify that `X+` rises while `X-` falls, and similarly for Y.
6. Confirm the differential voltage across `IN+` and `IN-` swings from −5 V to +5 V.
7. Only after this is correct, connect the laser with the arm switch and E-Stop circuits verified.

Never leave the laser connected and the arm switch ON during software or wiring changes.
