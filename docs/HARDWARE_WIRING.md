# Hardware Wiring Guide — Mosquito Laser Killer

**WARNING:** This system controls a **Class 4 laser** (2.5 W, 450 nm). Class 4 lasers cause instantaneous, irreversible eye injury and can ignite materials. Do not apply power to the laser until all enclosures, beam dumps, interlocks, and OD 4+ safety eyewear are in place. Read `AGENTS.md` before modifying code or wiring.

This document is the single source of truth for the **intended** stereoscopic
laser-targeting wiring. It matches the source code in `src/hal/mcp4922.cpp`,
`src/hal/differential_galvo_driver.cpp`, and
`src/control/coordinate_mapper.cpp`. The parts physically reported on hand are
tracked separately in [`HARDWARE_INVENTORY.md`](HARDWARE_INVENTORY.md).

> **Inventory no-go:** the currently reported stock is not a complete, validated
> build. In particular, 3.3 Ω resistors cannot replace the required 3.3 kΩ
> resistors; the 1 kΩ E-stop resistor and bipolar ±15 V galvo supply were not
> reported; switch contact ratings are unknown; and the generic four-channel
> I2C level shifter is not qualified for the complete SPI/laser signal set. Do
> not apply power until every reconciliation item in `HARDWARE_INVENTORY.md` is
> closed.

---

## Table of Contents

1. [System Block Diagram](#1-system-block-diagram)
2. [Inventory-Aware Wiring Bill of Materials](#2-inventory-aware-wiring-bill-of-materials)
3. [Mains Power & Physical E-Stop](#3-mains-power--physical-e-stop)
4. [DC Power Distribution](#4-dc-power-distribution)
5. [Arming Switch & GPIO 24 Sensing Circuit](#5-arming-switch--gpio-24-sensing-circuit)
6. [E-Stop GPIO Circuit](#6-e-stop-gpio-circuit)
7. [Raspberry Pi 5 GPIO Connections](#7-raspberry-pi-5-gpio-connections)
8. [SPI Bus & MCP4922 DAC Wiring](#8-spi-bus--mcp4922-dac-wiring)
9. [Logic-Level Translation](#9-logic-level-translation)
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

## 2. Inventory-Aware Wiring Bill of Materials

| Component | Required specification | Qty needed | Reported inventory / status |
|-----------|------------------------|------------|-----------------------------|
| Host | Raspberry Pi 5 | 1 | Project host |
| Cameras | OV9281 global-shutter monochrome, USB3 UVC | 2 | 2 reported on hand |
| Galvo kit | 20 kpps X-Y head, 400–700 nm; driver with ±15 VDC rails and ±5 V analog input | 1 | Reported on hand |
| Working laser | 2.5 W, 450 nm, 33 × 70 mm, 12 VDC, forced-air cooled, 3-pin TTL/PWM | 1 | Reported on hand; pinout/TTL levels unverified |
| Test laser | Low-power visible, electrically compatible with the complete gating chain | 1 | 5 mW, 12 mm module reported; electrical details/class label unverified |
| Laser PSU | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W | 1 | Reported on hand |
| Galvo PSU | Regulated bipolar ±15 VDC supply sized for both driver channels | 1 | **Not reported; required** |
| X-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Reported on hand |
| Y-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Reported on hand |
| SPI translation | Push-pull 3.3 V → 5 V translator/buffer qualified at the configured SPI rate for MOSI, SCLK, CE0, and CE1 | 4 channels | Generic 4-channel IIC/I2C module reported; **not yet qualified** |
| Laser translation | Dedicated, non-inverting 3.3 V → logic-supply interface with a verified fail-LOW power-up/down/open-wire state | 1 channel | **No qualified channel reported**; do not share an already-full four-channel SPI module |
| Monostable | 74HC123, DIP-16 dual retriggerable one-shot | 1 | Reported on hand; manufacturer/order code unverified |
| AND gate | SN74HC08N, DIP-14 quad 2-input AND | 1 | Reported on hand |
| Timing resistor | 220 kΩ, 1/2 W | 1 | Reported on hand |
| Timing capacitor | 1 µF, 50 V monolithic ceramic | 1 | Reported on hand; tolerance/effective capacitance unverified |
| Logic/DAC decoupling | 100 nF ceramic at each MCP4922, 74HC123, and SN74HC08 supply | 4 minimum | Value reported; available count unverified |
| Arm switch | Lever switch with contacts rated for the measured 12 VDC laser-driver load | 1 | Reported on hand; topology/rating unverified |
| E-stop | Mushroom actuator with 2 independent NC contacts; mains pole approved for the installation | 1 | Reported on hand; topology/rating unverified |
| Arm series resistor | 10 kΩ, 1/2 W | 1 | Reported on hand |
| E-stop series resistor | 1 kΩ, 1/2 W | 1 | **Not reported; required** |
| Sense pull-downs | 3.3 kΩ, 1/2 W | 2 | **Not reported; required. The stocked 3.3 Ω value is unusable here** |
| Zener clamps | BZX55C3V3, DO-35, 0.5 W | 2 | Part type reported; count unverified |
| Sense/debounce capacitors | 100 nF, 50 V monolithic ceramic | 2 | Value reported; count unverified |
| Distribution connectors | WAGO 221-413, 3-conductor, max 4 mm² | as required | Reported on hand; count unverified |
| Enclosure | Laser-safe interlocked case with beam dump | 1 | Required; status not reported |
| Safety eyewear | Correctly rated for 450 nm and the documented exposure analysis | 1 per person | Required; status not reported |

The stocked **3.3 Ω** resistors are intentionally not assigned a circuit role.
Meter every resistor before installation; colour-band confusion between 3.3 Ω
and 3.3 kΩ would keep both sense inputs LOW and make the documented interlocks
inoperable.

---

## 3. Mains Power & Physical E-Stop

The mushroom E-Stop is the last-line-of-defense physical disconnect. It kills AC power to both DC supplies.

```
Wall L ──► [E-STOP NC, mains-rated] ──► WAGO 221-413 (L) ─┬──► ±15 V supply AC L
                                           └──► 12 V supply AC L
Wall N ────────────────────────► WAGO 221-413 (N) ──────┬──► ±15 V supply AC N
                                           └──► 12 V supply AC N
PE/GND ────────────────────────► WAGO 221-413 (PE) ─────┬──► ±15 V supply PE
                                           └──► 12 V supply PE
```

### Important

- Before wiring, verify from markings or a datasheet that the on-hand mushroom
  assembly provides **two independent NC contacts** and that the mains pole is
  approved for the installation voltage/current. “Mushroom button” alone does
  not establish any of these properties.
- Wire only the **Live (Phase)** wire through the E-Stop. Neutral and Earth pass straight through.
- **Both independent contacts** are required:
  - Pole 1 carries the mains Live (AC side).
  - Pole 2 is used for the GPIO E-Stop sense circuit in Section 6.
- Use a separate WAGO 221-413 for each net. Never put Live, Neutral, PE, or a DC
  conductor into the same connector; a 221-413 joins all three ports together.
- Install the connectors in a suitable enclosure with strain relief and
  mains/SELV segregation. Mains work must be performed and inspected by a
  qualified person under the applicable local rules.
- Do not rely on software alone. The physical E-Stop must remove all power that can move the beam or fire the laser.

---

## 4. DC Power Distribution

| Supply | Output | Feeds | Notes |
|--------|--------|-------|-------|
| Bipolar galvo supply | ±15 VDC | Galvo driver board | Required by the reported driver; this supply was not reported in the current inventory |
| Mean Well LRS-50-12 | 12 VDC | Laser driver via arm switch | Keep laser on a switched branch; do not use this single-output supply as a substitute for ±15 VDC |

The Raspberry Pi 5 itself is powered from its 5 V USB-C supply. The 12 V supply
feeds only the laser driver module through the arm switch. Do not energize the
galvo driver until the correct bipolar ±15 VDC supply has been identified.

---

## 5. Arming Switch & GPIO 24 Sensing Circuit

The arm switch performs **two functions**: it switches 12 V to the laser driver
**and** it tells the Pi (via GPIO 24) that the system is armed. The laser cannot
fire unless the arm switch is ON and the software also asserts the GPIO 18 TTL
signal. Verify the on-hand lever switch's contact arrangement and DC load rating
before assigning it this role.

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
- **Do not fit the stocked 3.3 Ω part.** It would produce only about 4 mV at
  GPIO 24, so the Pi could never observe an armed state. Obtain a real 3.3 kΩ
  resistor and confirm it with a meter before soldering.
- **100 nF**: filters switch bounce and high-frequency noise.
- **BZX55C3V3**: cathode to the junction, anode to GND. Clamps transients above ~3.3 V.
- The arm switch must be rated for the laser driver current (typically < 1 A at 12 V).

---

## 6. E-Stop GPIO Circuit

The second independent NC contact on the mushroom E-stop also provides a
software-monitored E-stop on GPIO 25. The control thread polls this pin every
cycle; if it goes LOW, the system transitions to `SAFE_HALT`, forces the laser
LOW, and centers the galvos. Do not assume the on-hand button has this contact:
verify its contact blocks first.

```
3.3 V (RPi) ──► [E-Stop pole 2 NC] ──► 1 kΩ ──►┬──► GPIO 25 (RPi)
                                                   ├── 3.3 kΩ ──► GND
                                                   ├── 100 nF ──► GND
                                                   └── BZX55C3V3 cathode ──► GND
```

### Component notes

- **1 kΩ** series (NOT 10 kΩ): the source here is the Pi's own 3.3 V rail, not
  the 12 V of the arm-switch circuit, so the divider must be recomputed.
  Released-state node voltage: `3.3 V × 3.3 kΩ / (1 kΩ + 3.3 kΩ) ≈ 2.53 V` — a
  solid 3.3 V-logic HIGH. An earlier revision of this drawing copied the arm
  circuit's 10 kΩ, which from 3.3 V yields only ~0.82 V — below V_IH, so the
  input would read "pressed" permanently and the system could never leave the
  E-stop state. If your bench was built from that revision, replace the series
  resistor.
- **3.3 kΩ**: pull-down that forces LOW when the contact opens or a wire breaks.
- The reported inventory contains neither this 3.3 kΩ value nor the 1 kΩ series
  value. The stocked 3.3 Ω and 10 kΩ resistors are not substitutes; obtain and
  meter-check the specified parts before building this circuit.
- **100 nF** + **BZX55C3V3** (cathode to the junction): debounce and transient
  clamp, as in the arm circuit.

### Operation

| State | Pole 2 contact | GPIO 25 | Software interpretation |
|-------|----------------|---------|-------------------------|
| Released (normal) | Closed | ≈ 2.5 V (HIGH) | System OK |
| Pressed | Open | LOW (pulled down) | Emergency stop → SAFE_HALT |
| Wire broken | Open | LOW (pulled down) | Emergency stop → SAFE_HALT |

This is fail-safe: a broken wire or pressed button produces the same safe state.

---

## 7. Raspberry Pi 5 GPIO Connections

| Function | GPIO | Pin | Direction | Voltage | Destination |
|----------|------|-----|-----------|---------|-------------|
| Laser TTL | GPIO 18 | 12 | Output | 3.3 V → dedicated qualified fail-LOW interface | 74HC123/74HC08 backstop, then verified laser TTL input |
| Arm switch sense | GPIO 24 | 18 | Input | 2.98 V HIGH | Arm switch voltage divider |
| E-Stop sense | GPIO 25 | 22 | Input | ≈2.5 V HIGH | E-Stop NC + pull-down |
| SPI0 MOSI | GPIO 10 | 19 | Output | 3.3 V → qualified push-pull translator → 5 V | Both MCP4922 SDI |
| SPI0 MISO | GPIO 9 | 21 | Input | 3.3 V | Not used by MCP4922 (write-only) |
| SPI0 SCLK | GPIO 11 | 23 | Output | 3.3 V → qualified push-pull translator → 5 V | Both MCP4922 SCK |
| SPI0 CE0 | GPIO 8 | 24 | Output | 3.3 V → qualified push-pull translator → 5 V | MCP4922 #1 /CS (X-axis) |
| SPI0 CE1 | GPIO 7 | 26 | Output | 3.3 V → qualified push-pull translator → 5 V | MCP4922 #2 /CS (Y-axis) |
| 3.3 V | 3V3 | 1 / 17 | Power | 3.3 V | Pull-up, E-stop sense |
| 5 V | 5V | 2 / 4 | Power | 5 V | Qualified translator logic side, MCP4922 VDD, 74HC logic (subject to final interface design) |
| GND | GND | 6, 9, 14, 20, 25, 30, 34, 39 | Power | 0 V | Common ground |

---

## 8. SPI Bus & MCP4922 DAC Wiring

Two MCP4922 DACs share the SPI0 bus. The chip-select pins determine which axis receives the command.

```
RPi GPIO 10 (MOSI) ──► qualified 3.3 V→5 V push-pull translator ──► MCP4922 #1 SDI
RPi GPIO 10 (MOSI) ──► qualified 3.3 V→5 V push-pull translator ──► MCP4922 #2 SDI

RPi GPIO 11 (SCLK) ──► qualified 3.3 V→5 V push-pull translator ──► MCP4922 #1 SCK
RPi GPIO 11 (SCLK) ──► qualified 3.3 V→5 V push-pull translator ──► MCP4922 #2 SCK

RPi GPIO 8  (CE0) ──► qualified 3.3 V→5 V push-pull translator ──► MCP4922 #1 /CS
RPi GPIO 7  (CE1) ──► qualified 3.3 V→5 V push-pull translator ──► MCP4922 #2 /CS

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

Both DACs are powered from the **same 5 V rail** as the translator's 5 V logic
side. With a 5 V reference and unity gain, full-scale code `4095` is nominally
`4095/4096 × 5 V` (approximately 4.999 V), and mid-scale `2048` is 2.5 V.

---

## 9. Logic-Level Translation

The Raspberry Pi GPIO outputs 3.3 V logic. The MCP4922, when powered from 5 V,
specifies a minimum logic HIGH of `0.7 × VDD = 3.5 V`, so MOSI, SCLK, CE0, and
CE1 all require a guaranteed interface. Direct 3.3 V chip-select wiring is not
valid merely because it may appear to work on one bench unit.

The reported converter is a generic four-channel bidirectional **IIC/I2C**
module. Its IC/MOSFET topology, pull-up values, propagation delay, rise time,
and safe power-off behaviour are unknown. I2C modules are normally intended for
open-drain buses; do not claim 20 MHz push-pull SPI compatibility without a
schematic/datasheet and scope verification. Its four channels would already be
fully consumed by MOSI, SCLK, CE0, and CE1, leaving no channel for GPIO 18.

The final interface must therefore provide:

- four non-inverting, push-pull 3.3 V → 5 V channels qualified at
  `spi_speed_hz` for MOSI, SCLK, CE0, and CE1;
- a separate laser-control interface whose output is demonstrably LOW during
  Pi boot, power-up/down sequencing, an unpowered input side, and an open wire;
- common logic grounds and supply decoupling according to the selected devices'
  datasheets;
- scope captures showing valid HIGH/LOW levels and timing at each MCP4922, plus
  the laser-path tests in §11a.

Until a concrete translator/buffer and schematic satisfy those requirements,
Sections 8 and 11 describe required signal behaviour, not an assembly-ready
connection for the on-hand I2C module.

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
| 0 | 0 V | 4.9988 V | −4.9988 V | −15.148° |
| 2048 | 2.5000 V | 2.4988 V (code 2047) | +1.22 mV | +0.0037° |
| 4095 | 4.9988 V | 0 V | +4.9988 V | +15.148° |

Do **not** ground `IN-`. The system is designed for differential drive; grounding the negative input would halve the usable range and invalidate the coordinate mapping.

---

## 11. Laser Module Wiring

The reported 2.5 W, 450 nm module is 33 × 70 mm, uses forced-air cooling and an
external constant-current driver from 12 VDC, and exposes a seller-described
3-pin TTL-switch/PWM-power interface. The actual three-pin order, logic ground,
active polarity, and thresholds have not been recorded. Obtain and verify that
information before replacing the labelled functional blocks below with pin
numbers.

```
12 V supply + ──► [verified DC-rated ARM switch] ──► Laser driver +VIN
12 V supply − ────────────────────────► Laser driver GND

GPIO18 ──► [dedicated qualified fail-LOW interface]
       ──► [74HC123 + AND gate backstop, §11a] ──► verified Laser driver TTL-switch pin
Laser driver GND ────────────────────► Common GND
Laser cooling fan ───────────────────► powered whenever the module/driver is powered
```

### Safety notes

- The laser driver receives **no power** until the arm switch is ON. This is the hardware interlock.
- The TTL input is the **software trigger**, but it passes through the pulse-duration backstop of §11a — the laser fires only when GPIO 18 is HIGH, the arm switch is ON, the E-Stop is released, the one-shot has not timed out, and all software safety gates are satisfied.
- Do not use the advertised PWM power-control mode. The firing path must remain
  one sustained level: repeated PWM edges can retrigger the 74HC123 and defeat
  its duration cap.
- Do not energize the working module without its forced-air cooling operating.
- Keep the TTL line short and away from the laser power cable to reduce noise.
- The laser module chassis must be bonded to protective earth.

---

## 11a. Laser TTL Pulse-Duration Backstop (74HC123)

Every *software* mechanism that ends a laser pulse (max-pulse enforcement,
watchdog, E-stop poll) runs on the **control thread**. If that thread hangs with
GPIO 18 stuck HIGH, no software turns the laser off. This one-shot is the
**independent hardware duration backstop** — it is not on the control thread and
needs neither software nor an operator to act. Its bound is the measured
assembled pulse width, not the nominal R/C calculation. See AGENTS.md §4.1.

### Circuit

The laser TTL is gated by `translated GPIO18 AND one-shot-Q`. The one-shot is
triggered by the translated GPIO 18 rising edge; when it times out, the AND gate
force-drives the laser TTL LOW even if GPIO 18 is still HIGH.

```
Qualified fail-LOW interface output ─┬───────────────► 74HC08 gate 1 input 1A (pin 1)
                                     ├───────────────► 74HC123 1B (pin 2, +edge trigger)
                                     └───────────────► 74HC123 1CLR/1RD (pin 3)
74HC123 1A (pin 1) ─────────────────────────────────► GND (required for 1B rising trigger)
74HC123 active-HIGH 1Q (pin 13) ────────────────────► 74HC08 gate 1 input 1B (pin 2)
74HC08 gate 1 output 1Y (pin 3) ────────────────────► verified laser-driver TTL switch pin

Timing network (channel 1):
    74HC123  1Cext (pin 14) ──┬── 1 µF / 50 V monolithic ceramic ──┬── 1Rext/Cext (pin 15)
    74HC123  1Rext/Cext (pin 15) ── 220 kΩ ──► +5 V
    74HC123 VCC (pin 16) ──► +5 V ; GND (pin 8) ──► common GND
    100 nF ceramic directly across 74HC123 pins 16↔8
    Unused channel 2: hold 2CLR/2RD (pin 11) LOW; tie 2A/2B (pins 9/10) to defined levels;
                      leave 2Cext/2Rext-Cext (pins 6/7) open

AND gate:
    SN74HC08N VCC (pin 14) ──► +5 V ; GND (pin 7) ──► common GND
    100 nF ceramic directly across SN74HC08N pins 14↔7
    Tie every unused AND-gate input to a defined HIGH or LOW; leave unused outputs open
```

Pin 13 is the channel-1 **active-HIGH Q** output on the standard 74HC123 pinout;
pin 4 is its active-LOW complement and must not be used for the AND gate. Verify
the full marking on the on-hand IC against its manufacturer's datasheet before
wiring. Holding 1A LOW is required for a rising edge on 1B to trigger the
monostable. Tying 1CLR/1RD to the translated fire level holds the channel reset
while idle and allows the rising fire transition to arm/trigger it; this exact
behaviour must be included in the scope tests below.

### Timing

For a 74HC123 whose manufacturer specifies `K = 0.45` at 5 V and this
capacitance range:

`t_W ≈ K · R_ext · C_ext = 0.45 · 220 kΩ · 1 µF ≈ 99 ms`

The on-hand device's manufacturer/full ordering code and the R/C tolerances are
not recorded. The coefficient can differ by manufacturer, and the fitted
monolithic ceramic capacitor's effective capacitance can differ from its label.
**Measure the assembled circuit on a scope.** If the measured result is around
99 ms it sits just below the ~105 ms real software bound, so the hardware becomes
the binding limit and clips legitimate max-length pulses. Any change to the
timing network requires a new calculation and scope verification; do not rely on
the nominal marking alone.

### Verification (mandatory before connecting the Class 4 laser)

- [ ] **Short pulse passes through:** drive GPIO 18 HIGH for ~10 ms; the laser TTL must be HIGH for ~10 ms (not stretched to ~99 ms). If it stretches, Q is driving the TTL *alone* — the AND gate is missing or miswired. Fix before proceeding.
- [ ] **Stuck-HIGH is capped:** hold GPIO 18 HIGH indefinitely; the laser TTL must go LOW at the measured t_W and stay LOW.
- [ ] **Trigger pins verified:** confirm 1A is LOW, translated GPIO 18 reaches
  1B and 1CLR/1RD, and pin 13 (not pin 4) supplies active-HIGH Q to the AND gate.
- [ ] **Period measured** on a scope and recorded; matches intent (§ Timing above).
- [ ] **No PWM firing:** confirm the firing path drives a single sustained level per pulse — a retriggering burst within t_W would hold the output HIGH and defeat the cap.

---

## 12. Camera Wiring

The two reported OV9281 cameras connect to the Raspberry Pi 5 USB 3.0 ports.
Use **USB 3.0** ports (blue) for full bandwidth; USB 2.0 ports will not sustain
the validated high-rate 640×400 modes reliably.

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

- Tie the Raspberry Pi GND, DAC AGND, translator/buffer GND, laser driver GND,
  galvo driver signal GND, and all required 0 V returns to the documented common
  signal reference. Keep protective earth as a distinct safety conductor and
  follow the selected supplies' grounding requirements.
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

Ideally, complementary analogue channels behave as follows:

- `V+ = 5 V`, `V− = 0 V` → `V_diff = +5 V`
- `V+ = 0 V`, `V− = 5 V` → `V_diff = −5 V`
- `V+ = 2.5 V`, `V− = 2.5 V` → `V_diff = 0 V`

The actual MCP4922 transfer is `V(code) = code/4096 · Vref`, while the driver
writes channel B as `4095 − code`. Thus the available differential endpoints
are approximately ±4.9988 V (±15.148°), and commanded code 2048 pairs with 2047,
leaving a one-LSB +1.22 mV residual (~0.0037°). A direct DAC shutdown writes
2048 to both channels and produces exactly 0 V differential. The software limits
commands to **±15°** (`galvo_limits` in `config/system_config.yaml`).

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
3. Apply mains power only after the correct 12 V and bipolar ±15 V supplies,
   E-stop contacts, enclosure, and distribution wiring have been verified and
   inspected.
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

1. Build the Pi, DACs, qualified SPI translation, and galvo-driver electronics.
2. Connect the galvo drivers **only** — do not connect the galvo motors yet if you want to test the electronics without mechanical motion.
3. Power the Pi. Run `mosquito_laser_killer` with the arm switch OFF.
4. Verify the DAC outputs are near 2.5 V on all four channels at startup.
5. Apply a test target (or use a test mode) and verify that `X+` rises while `X-` falls, and similarly for Y.
6. Confirm the differential voltage across `IN+` and `IN-` swings from −5 V to +5 V.
7. Scope-verify the separate laser-control interface and §11a one-shot with no
   laser connected. If optical alignment is required later, use the reported
   5 mW/12 mm module only after its electrical interface and class label are
   known and compatible. Do not connect the Class 4 module during bench bring-up.

Never leave the laser connected and the arm switch ON during software or wiring changes.
