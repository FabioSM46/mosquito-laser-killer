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
> reported; and switch contact ratings are unknown. Level translation is now
> **specified** as 2 × SN74AHCT125N (§9) but the devices are on order, not on
> hand; the generic four-channel I2C level shifter is excluded from both the SPI
> and laser paths. The **safety contactor, latching START circuit, mains isolator
> and enclosure door interlock** of §3 and §6a are specified but neither
> purchased nor built, and the **galvo driver's differential input topology and
> common-mode range** (§10) are unverified — that last item invalidates §14 and
> the coordinate mapping if it turns out wrong. Do not apply power until every
> reconciliation item in `HARDWARE_INVENTORY.md` is closed.

---

## Table of Contents

1. [System Block Diagram](#1-system-block-diagram)
2. [Inventory-Aware Wiring Bill of Materials](#2-inventory-aware-wiring-bill-of-materials)
3. [Mains Power, Safety Contactor & E-Stop](#3-mains-power-safety-contactor--e-stop)
4. [DC Power Distribution](#4-dc-power-distribution)
5. [Arming Switch & GPIO 24 Sensing Circuit](#5-arming-switch--gpio-24-sensing-circuit)
6. [E-Stop GPIO Circuit](#6-e-stop-gpio-circuit)
6a. [Enclosure Door Interlock](#6a-enclosure-door-interlock)
7. [Raspberry Pi 5 GPIO Connections](#7-raspberry-pi-5-gpio-connections)
8. [SPI Bus & MCP4922 DAC Wiring](#8-spi-bus--mcp4922-dac-wiring)
9. [Logic-Level Translation](#9-logic-level-translation)
10. [Galvo Driver Connections](#10-galvo-driver-connections)
11. [Laser Module Wiring](#11-laser-module-wiring)
11a. [Laser TTL Pulse-Duration Backstop (74HC123)](#11a-laser-ttl-pulse-duration-backstop-74hc123)
12. [Camera Wiring](#12-camera-wiring)
13. [Grounding & Shielding](#13-grounding--shielding)
14. [How the ±5 V Differential Signal Works](#14-how-the-5-v-differential-signal-works)
15. [Power-Up / Power-Down Sequence](#15-power-up--power-down-sequence)
16. [Bench Testing Without the Laser](#16-bench-testing-without-the-laser)
17. [Build & Bring-Up Order](#17-build--bring-up-order)

---

## 1. System Block Diagram

Rendered drawings live in [`diagrams/`](diagrams/README.md), numbered in **reading**
order — not build order, which is §17:

| Drawing | Covers |
|---------|--------|
| [1 — system overview](diagrams/1%20-%20system-overview.png) | The whole signal chain end to end, and what each interlock actually removes |
| [2 — mains power distribution](diagrams/2%20-%20mains-power-distribution.png) | Isolator, K1 contactor, the latching control circuit, DC branches (§3, §4) |
| [3 — GPIO sense circuits](diagrams/3%20-%20gpio-sense-circuits.png) | Arm and E-stop sense dividers, Zener polarity, door interlock (§5, §6, §6a) |
| [4 — SPI level translation](diagrams/4%20-%20spi-level-translation.png) | AHCT125 #1, both MCP4922s with full pin assignment, galvo inputs (§8, §9, §10) |
| [5 — laser TTL safety chain](diagrams/5%20-%20laser-ttl-safety-chain.png) | AHCT125 #2, the 74HC123 one-shot, the AND gate, all three pull-downs (§9.3, §11, §11a) |

The drawings follow this document, not the other way round. Where they disagree,
this document is correct and the drawing is stale — fix the drawing, and never
build from a drawing that contradicts it.

```
                    ┌── CONTROL CIRCUIT (coil current only) ─────────────┐
                    │  E-STOP pole 1 NC ─► DOOR NC ─► (START ∥ K1 aux)   │
                    │                                        │           │
                    │                                    K1 coil         │
                    └────────────────────────────────────────┼───────────┘
                             latching: releasing the E-stop  │
                             does not restart anything ──────┘
                                                             ▼
Mains inlet ─► ISOLATOR / RCD / OCP ─► [K1, two poles] ─┬─► ±15 VDC ─► Galvo driver ─► X/Y galvo head
  (non-polarised inlet: BOTH live                       │
   conductors are switched, §3)                         └─► 12 VDC ─┬─► cooling fan (unswitched, §11)
                                                                    └─► ARM ─► DOOR NC ─► Laser driver ─► 2.5 W 450 nm

Separate USB-C 5 V ─► Raspberry Pi 5.  NOT on K1: the log, the GPIO 25 sense
                                       circuit and the 5 V logic rail that holds
                                       the fail-LOW pull-downs defined all
                                       survive an E-stop (§3).

     LEFT camera ──────►  ┌──────────────┐
                          │              │─SPI0───► AHCT125 #1 ─► 2× MCP4922 ─► X/Y galvo drivers
     RIGHT camera ─────►  │  Raspberry   │
                          │   Pi 5       │─GPIO18─► AHCT125 #2 ─► 74HC123 ∧ 74HC08 ─► Laser TTL
     ARM sense ────────►  │  GPIO24/25   │
     E-STOP sense ─────►  │              │
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
| Safety contactor **K1** | 2 main poles rated for the **combined cold-start inrush** of both DC supplies (LRS-50-12 alone specifies 45 A at 230 VAC), coil matched to the control circuit, ≥1 auxiliary NO for the seal-in path (§3) | 1 | **Not reported; required.** Replaces direct mains switching through the mushroom contact |
| START button | NO momentary, panel mount — the deliberate restart action (§3) | 1 | **Not reported; required** |
| STOP button | NC momentary in the coil path, for normal shutdown that is not an E-stop actuation (§3) | 1 | Recommended |
| Mains isolator + protection | Main isolator, overcurrent protection and RCD coordination per local rules, upstream of everything | 1 set | **Not reported; required.** Design and inspection by a qualified person |
| Door interlock switch | 2 independent NC contacts with **positive/direct opening action** (IEC 60947-5-1 Annex K), tool-required actuator (§6a) | 1 | **Not reported; required.** Referenced by the checklist but never previously specified |
| X-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Reported on hand |
| Y-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Reported on hand |
| SPI translation | SN74AHCT125N, PDIP-14 quad bus buffer — push-pull, non-inverting, TTL input thresholds; carries MOSI, SCLK, CE0, CE1 | 1 package (4 of 4 ch) | **Specified (§9); on order.** Must be marked `AHCT` — not `AHC`, not `HC` |
| Laser translation | SN74AHCT125N, a **second physically separate** package, 1 channel used, with the two fail-LOW pull-downs of §9.3 | 1 package (1 of 4 ch) | **Specified (§9); on order.** Package #1 is fully consumed by SPI, so this is a second device, not a spare channel |
| Monostable | 74HC123, DIP-16 dual retriggerable one-shot | 1 | Reported on hand; manufacturer/order code unverified |
| AND gate | SN74HC08N, DIP-14 quad 2-input AND | 1 | Reported on hand |
| Timing resistor | 220 kΩ, 1/2 W | 1 | Reported on hand |
| Timing capacitor | 1 µF, 50 V monolithic ceramic | 1 | Reported on hand; tolerance/effective capacitance unverified |
| Logic/DAC decoupling | 100 nF ceramic directly across the supply pins of each MCP4922, SN74AHCT125N, 74HC123, and SN74HC08 | 6 minimum | Value reported; available count unverified |
| Rail bulk decoupling | 10 µF on the 5 V rail at the logic board | 1 | Not reported; required |
| Arm switch | Lever switch with contacts rated for the measured 12 VDC laser-driver load | 1 | Reported on hand; topology/rating unverified |
| E-stop | Mushroom actuator with 2 independent NC contacts — pole 1 in the K1 coil circuit, pole 2 for GPIO 25 (§3, §6). A monitored-safety-relay architecture would need a third | 1 | Reported on hand; topology/rating unverified |
| Arm series resistor | 10 kΩ, 1/2 W | 1 | Reported on hand |
| Laser-path fail-LOW pull-downs | 10 kΩ, 1/2 W — **three**, one per node: (a) translator input, (b) '123/'08 inputs, (c) laser-driver connector (§9.3) | 3 | Value reported on hand; count unverified |
| Chip-select pull-ups | 10 kΩ, 1/2 W to **3.3 V** on the CE0 and CE1 translator inputs (§8) | 2 | Value reported on hand; count unverified |
| E-stop series resistor | 1 kΩ, 1/2 W | 1 | **Not reported; required** |
| Sense pull-downs | 3.3 kΩ, 1/2 W | 2 | **Not reported; required. The stocked 3.3 Ω value is unusable here** |
| Zener clamps | BZX55C3V3, DO-35, 0.5 W — cathode at the sense junction, anode to GND | 2 | Part type reported; count unverified |
| Sense/debounce capacitors | 100 nF, 50 V monolithic ceramic | 2 | Value reported; count unverified |
| Mains terminal blocks | DIN-rail terminal blocks, one block per net: unswitched A/B, switched A/B, PE (§3) | 1 strip | **Not reported; required** |
| Distribution connectors | WAGO 221-413, 3-conductor, max 4 mm² — splicing connectors, **not** terminal blocks and not a barrier between circuits | as required | Reported on hand; count unverified |
| Enclosure | Laser-safe interlocked case with beam dump | 1 | Required; status not reported |
| Safety eyewear | Correctly rated for 450 nm and the documented exposure analysis | 1 per person | Required; status not reported |

**Running totals for the parts that appear in more than one circuit.** These are
the counts to shop against; the rows above are the counts per role.

| Value | Total | Where |
|-------|-------|-------|
| 10 kΩ, 1/2 W | **6** | 1 arm series + 3 laser-path fail-LOW (a)(b)(c) + 2 chip-select pull-ups |
| 100 nF, 50 V ceramic | **8** | 6 IC decoupling (2× MCP4922, 2× AHCT125, 74HC123, SN74HC08) + 2 sense debounce |
| 3.3 kΩ, 1/2 W | **2** | arm sense divider + E-stop sense pull-down |
| 1 kΩ, 1/2 W | **1** | E-stop series |
| BZX55C3V3 | **2** | one clamp per sense junction |
| 10 µF | **1** | 5 V rail bulk |
| 220 kΩ, 1/2 W | **1** | 74HC123 timing, to **pin 14** (§11a) |
| 1 µF, 50 V ceramic | **1** | 74HC123 timing, pins 15↔14 (§11a) |

The stocked **3.3 Ω** resistors are intentionally not assigned a circuit role.
Meter every resistor before installation; colour-band confusion between 3.3 Ω
and 3.3 kΩ would keep both sense inputs LOW and make the documented interlocks
inoperable.

The same hazard applies to the logic family: `AHCT`, `AHC`, and `HC` share the
same pinout and differ by one letter in the marking, but only `AHCT` accepts a
3.3 V input at a 5 V supply (§9.1). Read the marking on every fitted device.

**Sockets and permanence.** DIP sockets add nothing on a breadboard — the
breadboard is the socket — but on the soldered build they let you swap the
74HC123 without desoldering, which matters because its timing coefficient is
vendor-dependent and may need a different device or R/C. Note the trade: cheap
stamped-pin sockets fail intermittently, and an intermittent contact on the
74HC123 `1Q` output or the laser-path translator removes the pulse-duration
backstop **silently**. Use machined/turned-pin sockets on the laser path or
solder those devices directly. For the same reason the §11a backstop must not
live permanently on a breadboard: its entire job is to work at the moment the
control thread has hung with GPIO 18 HIGH, and a 95%-reliable contact makes that
a coin flip. Budget a soldered board before either laser is connected.

---

## 3. Mains Power, Safety Contactor & E-Stop

The E-stop must remove all power that can move the beam or fire the laser. It
does **not** do that by carrying the mains itself. A latching contactor does the
switching; the mushroom button breaks the contactor's coil circuit.

> **This section replaces a direct-switching arrangement that was wrong in two
> independent ways.** Both are recorded here so the mistake is not reintroduced.
>
> 1. **Inrush.** The Mean Well LRS-50-12 specifies a **45 A cold-start inrush at
>    230 VAC**; the bipolar ±15 V supply adds its own. A mushroom button's
>    contact block is a pilot-duty part rated for a fraction of that. The failure
>    mode is not a blown contact — it is a **welded** one, which leaves an E-stop
>    that looks and feels normal and disconnects nothing.
> 2. **Automatic restart.** Passing mains directly through the NC contact means
>    *releasing* the mushroom re-energises everything. Restoring power must
>    require a separate deliberate action. See ISO 13850 for the E-stop function
>    itself; the same principle is stated in Machinery Directive 2006/42/EC
>    Annex I §1.2.4.3 (superseded by Regulation (EU) 2023/1230 from 20 January
>    2027). A one-off self-built rig is not machinery placed on the market, so
>    neither instrument is a legal obligation here — they are cited as the design
>    standard, and the engineering reason stands on its own: **this system's own
>    documented recovery from an E-stop is "restart the software," because
>    `SAFE_HALT` is terminal.** An operator who does that with the mushroom
>    released and the arm switch still ON walks `INIT → IDLE → ARMED → TRACKING
>    → FIRING` with no deliberate start action anywhere in the sequence.

### Control circuit — contactor coil only, carries coil current

```
Switched conductor A ──► [E-STOP pole 1, NC] ──► [DOOR INTERLOCK, NC, §6a] ──┐
                                                                             │
                                     ┌── [START, NO momentary] ──────────────┤
                                     │              ∥                        │
                                     │   [K1 auxiliary NO — seal-in]         │
                                     └───────────────────────────────────────┤
                                                                             │
                                                                        [K1 coil]
                                                                             │
Conductor B ─────────────────────────────────────────────────────────────────┘
```

The START button and K1's own auxiliary NO contact sit in **parallel**. Pressing
START energises the coil; the auxiliary contact then closes and holds the coil
energised after START is released. Anything that breaks the series path — E-stop,
door, or loss of mains — drops K1, which opens the auxiliary contact and destroys
the hold path. **Nothing re-energises K1 except another deliberate START press.**

### Power circuit — K1 main contacts, double-pole

```
Conductor A ──► [K1 main pole 1] ──► switched A ──┬──► ±15 VDC galvo supply
Conductor B ──► [K1 main pole 2] ──► switched B ──┘   └──► 12 VDC LRS-50-12 (laser branch)

PE ──── never switched, never fused ──────────────────► both supply PE terminals,
                                                        enclosure, laser chassis,
                                                        galvo chassis (§13)
```

**Both live conductors are broken, not just "L".** If the equipment connects
through a non-polarised inlet — CEI 23-50 (Italian Type L), Schuko/CEE 7,
and others are all reversible — then the conductor you labelled "L" is Live only
half the time it is plugged in. Single-pole switching on such an inlet can break
Neutral and leave both supplies live-referenced behind an E-stop that reads as
open. This is why the previous "wire only the Live wire through the E-Stop"
instruction has been removed. PE is never switched and never fused.

### Behaviour

| Event | K1 | Galvo ±15 V and laser 12 V | To restore |
|-------|----|-----------------------------|------------|
| Mains applied at the isolator | de-energised | **OFF** | press START |
| START pressed, then released | energised, sealed in | ON | — |
| E-stop pressed | drops out | **OFF** | release E-stop, **then** press START |
| E-stop released | stays de-energised | **OFF** | press START |
| Enclosure door opened (§6a) | drops out | **OFF** | close door, **then** press START |
| Mains dip or interruption | drops out | **OFF** | press START |

The Raspberry Pi is **not** on the contactor. It runs from its own USB-C supply
so the control software, the GPIO 25 sense circuit (§6) and the 5 V logic rail
that holds the laser-path pull-downs defined (§9.3) all survive an E-stop. That
is deliberate: an E-stop should remove the hazard and leave the diagnostics
running.

### Residual risk this architecture does *not* remove

Moving the switching into K1 removes the weld hazard from the mushroom's contact
and moves it to K1's main poles. A welded main pole means the E-stop drops the
coil and the supplies stay live. Two mitigations, in increasing order of
integrity:

- **Minimum:** specify K1 with main contacts rated for the combined inrush (not
  the steady-state current), and add "press E-stop, confirm both DC rails are
  actually at 0 V" to the periodic checks — measured, not assumed.
- **Higher integrity:** a monitored safety relay with dual-channel E-stop inputs
  and feedback from a **mirror contact** on K1 (IEC 60947-4-1: an NC auxiliary
  guaranteed not to be closed while any main NO pole is closed). This detects the
  welded pole and refuses the reset. It needs two NC E-stop contacts for the
  safety function, so the mushroom would need three contacts in total — or the
  GPIO 25 sense of §6 moves to a spare auxiliary on K1, which is arguably a
  better signal anyway: it reports *power actually removed* rather than *button
  pressed*.

Choosing between these is a functional-safety decision, not a wiring detail. It
is an open no-go item in `HARDWARE_INVENTORY.md`.

### Required, and not yet specified or purchased

- Contactor **K1**: coil voltage matched to the control circuit, **two** main
  poles rated for the combined cold-start inrush of both supplies, and at least
  one auxiliary NO for the seal-in path.
- **START** (NO momentary) and, optionally, a **STOP** (NC momentary) in series
  with the coil path for normal shutdown that is not an E-stop actuation.
- Upstream **main isolator**, **overcurrent protection**, and **RCD**
  coordination per local rules.
- Enclosure **door interlock** with two NC contacts (§6a).

### Wiring practice

- The mushroom still needs **two independent NC contacts**: pole 1 in the coil
  circuit above, pole 2 for the GPIO 25 sense circuit of §6. Verify from markings
  or datasheet that the on-hand assembly has them. Pole 1's duty is now only the
  coil current, which is far easier to satisfy than the original mains duty — but
  it still must be rated for it.
- Use **DIN-rail terminal blocks**, one block per net: unswitched A, unswitched
  B, switched A, switched B, PE. WAGO 221-413 lever connectors are three-port
  *splicing* connectors — every port in one connector is the same net, so they
  are not a substitute for a terminal strip and never a barrier between circuits.
  If you use them, it is one connector per net with nothing shared.
- Mains and SELV wiring segregated, strain-relieved, inside an enclosure.
- Mains work must be designed, performed and inspected by a qualified person
  under the applicable local rules. Nothing in this document substitutes for
  that.

---

## 4. DC Power Distribution

| Supply | Output | On K1? | Feeds | Notes |
|--------|--------|--------|-------|-------|
| Bipolar galvo supply | **+15 V, 0 V/COM, −15 V** — three conductors, not "±15 V" as one wire | Yes | Galvo driver board | Required by the reported driver; not reported in the current inventory. COM is the reference for both rails and bonds to the common ground (§13) |
| Mean Well LRS-50-12 | 12 VDC | Yes | Laser cooling fan directly; laser driver via arm switch + door interlock | Do not use this single-output supply as a substitute for the bipolar ±15 VDC |
| Pi USB-C supply | 5 VDC | **No** | Raspberry Pi 5 | Deliberately not on the contactor (§3) |
| 5 V logic rail | 5 VDC, from Pi header pins 2/4 | **No** | Both SN74AHCT125N, both MCP4922 VDD/Vref, 74HC123, SN74HC08 | Derived from the Pi, so it tracks the Pi and survives an E-stop |

Two consequences of that last row are load-bearing:

- **The 5 V logic rail survives an E-stop.** The three fail-LOW pull-downs of
  §9.3, the AND gate and the one-shot all stay powered and defined while the
  galvo ±15 V and laser 12 V are dead. An E-stop removes the hazard without
  leaving the control chain in an undefined state.
- **The logic rail cannot be live while the Pi is off.** That is why `MOSI` and
  `SCLK` need no external bias (§8) — their translator inputs cannot float while
  the buffer is powered.

Budget the rail: two AHCT125s, two MCP4922s, a 74HC123 and a 74HC08 together draw
well under 50 mA, comfortably inside the Pi 5 header's 5 V capability. Fit the
10 µF bulk capacitor at the logic board where the rail arrives, in addition to the
per-IC 100 nF.

Do not energize the galvo driver until the correct bipolar supply has been
identified **and** its input topology verified (§10).

---

## 5. Arming Switch & GPIO 24 Sensing Circuit

The arm switch performs **two functions**: it switches 12 V to the laser driver
**and** it tells the Pi (via GPIO 24) that the system is armed. The laser cannot
fire unless the arm switch is ON and the software also asserts the GPIO 18 TTL
signal. Verify the on-hand lever switch's contact arrangement and DC load rating
before assigning it this role.

```
12 V + (switched by K1, §3) ──► [ARM SPST] ──► [DOOR INTERLOCK NC, §6a] ──┐
                                                                          │
                                                       Laser driver +VIN ◄─┤
                                                                          │
                              ┌───────────────────── 10 kΩ ◄──────────────┘
                              │
              GPIO 24 ◄───────┼── junction ──┬── 3.3 kΩ ─────► GND
            (RPi pin 18)      │              ├── 100 nF ─────► GND
                              └──────────────┴── BZX55C3V3 ──► GND
                                                 banded (cathode) end at the
                                                 junction, anode at GND
```

### Component notes

- **10 kΩ**: current-limiting series resistor from the 12 V arm signal.
- **3.3 kΩ**: lower leg of the voltage divider, **connected to the same junction**
  as the 10 kΩ, the 100 nF, the Zener and the GPIO. GPIO 24 sees
  `12 V × 3.3 kΩ / (10 kΩ + 3.3 kΩ) ≈ 2.98 V` — a valid 3.3 V logic HIGH.
  Without it there is no divider: the GPIO pin sees 12 V through 10 kΩ with the
  Zener as the only thing standing between the rail and a 3.3 V input, drawing
  about 0.9 mA through it continuously. **It still reads HIGH**, which is what
  makes the omission dangerous — the circuit appears to work until the Zener
  fails.
- **Do not fit the stocked 3.3 Ω part.** It would produce only about 4 mV at
  GPIO 24, so the Pi could never observe an armed state. Obtain a real 3.3 kΩ
  resistor and confirm it with a meter before soldering.
- **100 nF**: filters switch bounce and high-frequency noise.
- **BZX55C3V3**: **cathode (the banded end) at the junction, anode at GND.** It
  clamps transients above ~3.3 V. Fitted backwards it is a forward-biased diode
  that pins the junction near 0.7 V, so the Pi can never read armed — fail-safe,
  but silent and baffling. Confirm the band before soldering; a diode-test on a
  multimeter reads the polarity in seconds.
- The arm switch must be rated for the laser driver current (typically < 1 A at
  12 V).
- **The sense tap is downstream of the door interlock, deliberately.** GPIO 24
  then means "the laser driver actually has power available," not merely "the
  lever is up": opening the enclosure door reads as a *disarm*, which the
  existing `ArmSwitch` → `FiringController::set_armed(false)` path already handles
  by clearing targets and rejecting fire. The door interlock becomes visible to
  the software through a path that already exists, with no code change and no
  additional GPIO.

---

## 6. E-Stop GPIO Circuit

The second independent NC contact on the mushroom E-stop also provides a
software-monitored E-stop on GPIO 25. The control thread polls this pin every
cycle; if it goes LOW, the system transitions to `SAFE_HALT`, forces the laser
LOW, and centers the galvos. Do not assume the on-hand button has this contact:
verify its contact blocks first.

```
3.3 V (RPi) ──► [E-Stop pole 2, NC] ──► 1 kΩ ──┐
                                               │
              GPIO 25 ◄──────── junction ──────┼── 3.3 kΩ ─────► GND
            (RPi pin 22)            │          ├── 100 nF ─────► GND
                                   └──────────┴── BZX55C3V3 ──► GND
                                                  banded (cathode) end at the
                                                  junction, anode at GND
```

All four legs — the 1 kΩ from the E-stop contact, the 3.3 kΩ, the 100 nF, the
Zener and the wire to GPIO 25 — meet at **one** junction. The 3.3 kΩ is what
pulls the pin LOW when the contact opens; wired anywhere else there is no
pull-down and the fail-safe property in the table below does not exist.

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
- **100 nF** + **BZX55C3V3**: debounce and transient clamp, as in the arm
  circuit. **Cathode (banded end) at the junction, anode at GND.** Reversed, it
  forward-clamps the junction near 0.7 V — below `V_IH` — so the pin reads
  "pressed" permanently and the system can never leave `SAFE_HALT`.
- **Meter both sense networks with the GPIO wire disconnected**, before either is
  attached to the Pi header. Arm switch OFF/ON must give ≈ 0 V / 2.98 V; E-stop
  pressed/released must give ≈ 0 V / 2.53 V. A wiring error found with a meter
  costs a minute; the same error found by the Pi can cost a GPIO pin.

### Operation

| State | Pole 2 contact | GPIO 25 | Software interpretation |
|-------|----------------|---------|-------------------------|
| Released (normal) | Closed | ≈ 2.5 V (HIGH) | System OK |
| Pressed | Open | LOW (pulled down) | Emergency stop → SAFE_HALT |
| Wire broken | Open | LOW (pulled down) | Emergency stop → SAFE_HALT |

This is fail-safe: a broken wire or pressed button produces the same safe state.

---

## 6a. Enclosure Door Interlock

A Class 4 enclosure must terminate access to the beam when it is opened. Earlier
revisions of this project referenced door-interlock *testing*
(`PRE_FLIGHT_CHECKLIST.md` §8) without ever specifying a circuit, which meant the
interlock existed only as an intention. This section is that circuit.

The interlock uses a switch with **two independent NC contacts**, one in each of
two paths that are already load-bearing:

```
Contact 1 ──► in series with the E-stop in the K1 coil circuit (§3)
              door open  →  K1 drops out  →  ±15 V and 12 V both removed
                        →  restoring power requires a deliberate START press

Contact 2 ──► in series with the arm switch on the 12 V laser feed (§5)
              door open  →  laser driver +VIN removed directly, independently
                        of K1, and GPIO 24 reads disarmed (§5)
```

Contact 1 removes everything that can move the beam or fire the laser. Contact 2
removes laser-driver power on its own path, so a welded K1 main pole (§3) does not
also defeat the door interlock. They are deliberately not redundant copies of each
other: they fail independently.

### This is a hardware-only interlock, and that is the stronger choice

There is no door GPIO and no code change. An interlock's job is to **remove the
hazard**, not to inform software, and a contact that opens the 12 V feed is
strictly stronger than a pin the control thread polls — the same reasoning
`AGENTS.md` §4.8 already applies to the arm switch, which is called out there as
"a true hardware interlock." Adding a software-visible door sense would mean a new
GPIO, a new HAL interface, a control-loop poll, state-machine wiring and tests, in
exchange for a signal that cannot do anything the contacts have not already done.

It is nonetheless visible to the software for free: contact 2 sits upstream of the
GPIO 24 sense tap, so opening the door reads as a disarm and the existing
`FiringController::set_armed(false)` path clears targets and rejects fire (§5).

### Switch requirements

- **Two independent NC contacts**, both with **positive/direct opening action**
  (IEC 60947-5-1 Annex K), so that a welded contact is still forced open by the
  door movement rather than relying on a spring.
- Mounted so the actuator cannot be operated by hand while the door is open
  without a tool — an interlock that can be held closed with a fingertip is not
  an interlock.
- Actuated by the door itself, on the hinge-remote side, so a warped or partly
  closed door does not read as closed.

### Operational consequence

Every door opening drops the contactor, so closing the door and pressing START is
part of normal operation. That is the intended behaviour, not an inconvenience to
engineer around: any work needing the enclosure open is work that must not have
hazardous power available. Alignment is done with a verified low-power source
inside the **closed** enclosure (`PRE_FLIGHT_CHECKLIST.md` §5), which is why the
interlock does not obstruct the procedure it protects.

---

## 7. Raspberry Pi 5 GPIO Connections

| Function | GPIO | Pin | Direction | Voltage | Destination |
|----------|------|-----|-----------|---------|-------------|
| Laser TTL | GPIO 18 | 12 | Output | 3.3 V → SN74AHCT125N **#2** ch1 (§9) | 74HC123/74HC08 backstop, then verified laser TTL input |
| Arm switch sense | GPIO 24 | 18 | Input | 2.98 V HIGH | Arm switch voltage divider |
| E-Stop sense | GPIO 25 | 22 | Input | ≈2.5 V HIGH | E-Stop NC + pull-down |
| SPI0 MOSI | GPIO 10 | 19 | Output | 3.3 V → SN74AHCT125N **#1** ch1 → 5 V | Both MCP4922 SDI (pin 5) |
| SPI0 MISO | GPIO 9 | 21 | Input | 3.3 V | Not used by MCP4922 (write-only) |
| SPI0 SCLK | GPIO 11 | 23 | Output | 3.3 V → SN74AHCT125N **#1** ch2 → 5 V | Both MCP4922 SCK (pin 4) |
| SPI0 CE0 | GPIO 8 | 24 | Output | 3.3 V → SN74AHCT125N **#1** ch3 → 5 V | MCP4922 #1 /CS (X-axis) |
| SPI0 CE1 | GPIO 7 | 26 | Output | 3.3 V → SN74AHCT125N **#1** ch4 → 5 V | MCP4922 #2 /CS (Y-axis) |
| 3.3 V | 3V3 | 1 / 17 | Power | 3.3 V | Pull-up, E-stop sense |
| 5 V | 5V | 2 / 4 | Power | 5 V | Both SN74AHCT125N, MCP4922 VDD/Vref, 74HC123, SN74HC08 |
| GND | GND | 6, 9, 14, 20, 25, 30, 34, 39 | Power | 0 V | Common ground |

---

## 8. SPI Bus & MCP4922 DAC Wiring

Two MCP4922 DACs share the SPI0 bus. The chip-select pins determine which axis receives the command.

```
RPi pin 19  GPIO 10 (MOSI) ──► AHCT125 #1 ch1 ──┬──► MCP4922 #1 SDI (pin 5)
                                                └──► MCP4922 #2 SDI (pin 5)
RPi pin 23  GPIO 11 (SCLK) ──► AHCT125 #1 ch2 ──┬──► MCP4922 #1 SCK (pin 4)
                                                └──► MCP4922 #2 SCK (pin 4)
RPi pin 24  GPIO 8  (CE0) ─┬─► AHCT125 #1 ch3 ─────► MCP4922 #1 /CS (pin 3)   X-axis
                           └── 10 kΩ ──► +3.3 V
RPi pin 26  GPIO 7  (CE1) ─┬─► AHCT125 #1 ch4 ─────► MCP4922 #2 /CS (pin 3)   Y-axis
                           └── 10 kΩ ──► +3.3 V

RPi 5 V ───────────────────────────────────────────► both MCP4922 VDD + VREFA/VREFB
RPi 5 V ───────────────────────────────────────────► both MCP4922 /SHDN (pin 9)
GND (common) ──────────────────────────────────────► both MCP4922 AVSS
GND (common) ──────────────────────────────────────► both MCP4922 /LDAC (pin 8)
```

### Chip-select boot state

The two 10 kΩ pull-ups to **3.3 V** (translator input side, not the 5 V side)
hold both `/CS` lines deasserted from the moment the 5 V rail appears until the
application configures SPI0. GPIO 7 and GPIO 8 happen to default to internal
pull-ups on this SoC, but that is a convention of the boot firmware, not a
guarantee this project should rest a DAC write on.

`MOSI` and `SCLK` deliberately get **no** external bias. They cannot float while
the buffer is powered: the 5 V logic rail is taken from the Pi header (§7), so
the translator is energised only when the Pi is, and GPIO 9–27 default to
internal pull-downs at reset. Adding resistors there would be two more parts
guarding a state that cannot occur — and with `/CS` held HIGH no write can be
latched regardless of what MOSI and SCLK do.

### MCP4922 pin assignment (PDIP-14)

Identical for both packages except `/CS`. Confirm against the
[MCP4922 data sheet](https://ww1.microchip.com/downloads/en/devicedoc/22250a.pdf)
before soldering.

| Pin | Name | Connect to |
|-----|------|------------|
| 1 | VDD | +5 V |
| 2 | NC | leave open |
| 3 | /CS | translated CE0 (X-axis DAC) / CE1 (Y-axis DAC) |
| 4 | SCK | translated SCLK |
| 5 | SDI | translated MOSI |
| 6, 7 | NC | leave open |
| 8 | /LDAC | **GND** |
| 9 | /SHDN | **+5 V** |
| 10 | VOUTB | this axis' galvo driver IN− (§10) |
| 11 | VREFB | +5 V |
| 12 | AVSS | common GND |
| 13 | VREFA | +5 V |
| 14 | VOUTA | this axis' galvo driver IN+ (§10) |

Plus 100 nF ceramic directly across pins 1↔12 on each package.

### /LDAC and /SHDN are not optional

Both are CMOS inputs with no internal pull; left floating their state is
indeterminate and the resulting failure is **silent**. Earlier revisions of this
document omitted them entirely.

- **`/LDAC` (pin 8) → GND.** This transfers each channel's serial latch to the
  output on the rising edge of `/CS`. Left HIGH or floating HIGH, writes never
  reach the output latch: `MCP4922::write()` returns success, the SPI traffic
  looks correct on a scope, and the outputs never move.
  `src/hal/mcp4922.cpp` relies on the `/CS`-rising-edge update and issues no
  separate `/LDAC` strobe.
- **`/SHDN` (pin 9) → +5 V.** This keeps the output stage active. Pulled or
  floating LOW, the outputs disconnect and the axis is dead.

### Vref tied to the 5 V rail

Tying `VREFA`/`VREFB` to VDD is valid **only because the driver writes BUF = 0**
(unbuffered reference), whose input range is 0 V to VDD. The command words in
`src/hal/mcp4922.h` set BUF = 0 and GA = 1 (unity gain). If BUF is ever set,
Vref = VDD leaves the specified buffered-reference range and this wiring becomes
invalid.

Because Vref *is* the Pi's 5 V rail, `dac_reference_voltage` in
`config/system_config.yaml` must hold the **measured** rail voltage at the DAC
pin under load, not the nominal 5.0. `CoordinateMapper` divides by that number,
so a 2 % error is 2 % of commanded angle — about 0.3° at full scale, ≈4 mm at
0.75 m, against a 5 mm target.

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

The Raspberry Pi GPIO outputs 3.3 V logic. Two families of 5 V device sit
downstream and **neither** is guaranteed to accept it:

| Downstream input | Supply | Minimum guaranteed logic HIGH |
|------------------|--------|-------------------------------|
| MCP4922 SDI, SCK, /CS | 5 V | `0.7 × VDD` = **3.5 V** |
| 74HC123, SN74HC08N | 5 V | `0.7 × VCC` = **3.5 V** |

3.3 V does not meet 3.5 V. Direct wiring — including direct 3.3 V chip-select
drive — is not valid merely because it may appear to work on one bench unit.

### 9.1 Selected translator: 2 × SN74AHCT125N

`AHCT` logic has **TTL** input thresholds (`V_IH` = 2.0 V at `VCC` = 5 V), so a
3.3 V Pi output drives it with margin, and its outputs are 5 V push-pull rated
far beyond the configured `spi_speed_hz`.

> **`AHCT` is load-bearing.** `AHC` and `HC` share the identical pinout but use
> CMOS thresholds (`0.7 × VCC` = 3.5 V) and reproduce the exact problem this part
> exists to solve. A wrong-family substitution is invisible on the assembled
> board. Read the marking on every fitted device.

Two **physically separate** packages are used:

| Package | Channels used | Carries |
|---------|---------------|---------|
| #1 | 4 of 4 | SPI0 MOSI, SCLK, CE0, CE1 |
| #2 | 1 of 4 | GPIO 18 laser TTL |

Package #2 is required, not a precaution: #1's four channels are fully consumed
by SPI, leaving no channel for GPIO 18. The separation additionally means a die
fault or ESD damage in the SPI package cannot take the laser path with it, and a
miswire in the SPI harness cannot physically land on the laser channel. Both
packages share the 5 V rail, so this is **not** supply isolation — do not record
it as such in the FMEA.

### 9.2 SN74AHCT125N pin assignment (PDIP-14)

Confirm against the manufacturer's datasheet before soldering.

| Pin | Name | Package #1 (SPI) | Package #2 (laser) |
|-----|------|------------------|--------------------|
| 14 | VCC | +5 V | +5 V |
| 7 | GND | common GND | common GND |
| 1, 4, 10, 13 | 1–4 /OE | all → GND | all → GND |
| 2 / 3 | 1A / 1Y | Pi pin 19 MOSI → both MCP4922 SDI | GPIO 18 → §11a backstop |
| 5 / 6 | 2A / 2Y | Pi pin 23 SCLK → both MCP4922 SCK | unused |
| 9 / 8 | 3A / 3Y | Pi pin 24 CE0 → X-axis MCP4922 /CS | unused |
| 12 / 11 | 4A / 4Y | Pi pin 26 CE1 → Y-axis MCP4922 /CS | unused |

- 100 nF ceramic directly across pins 14↔7 on **each** package.
- Tie every unused `A` input to GND. Never leave a CMOS input floating.
- `/OE` is tied LOW permanently and deliberately. It is **not** a laser enable —
  see §9.3 for why the Hi-Z state must be handled passively instead.

### 9.3 The laser path must fail LOW at every stage, not just the first

The laser control line must be demonstrably LOW during Pi boot, power-up/down
sequencing, an unpowered logic rail, a missing IC, and any open wire. No single
pull-down achieves that, because each one only defends the node it sits on. The
chain has three nodes, so it needs **three** pull-downs — each one physically
adjacent to the input it protects, not at a convenient point on the board:

```
Pi pin 12 (GPIO 18) ──┬──► AHCT125 #2 1A (pin 2)
                      └── 10 kΩ ──► GND                                    (a)
                          fitted at the translator input

AHCT125 #2 1Y (pin 3) ┬──► 74HC123 1B + 1CLR/1RD, 74HC08 1A (§11a)
                      └── 10 kΩ ──► GND                                    (b)
                          fitted beside the 74HC123 / 74HC08 inputs

74HC08 1Y (pin 3) ────┬──► verified laser-driver TTL-switch pin
                      └── 10 kΩ ──► GND                                    (c)
                          fitted at the laser-driver connector
```

| Failure case | Covered by |
|--------------|-----------|
| Broken wire between the Pi header and the translator input | (a) |
| Pi not yet booted, or GPIO 18 released back to an input | (a), plus GPIO 18's internal pull-down |
| Translator output Hi-Z: package unpowered, `/OE` high, or **IC removed from its socket** | **(b) only** |
| 5 V logic rail down, or 74HC123 removed, leaving `1Q` floating | (b) holds the AND input LOW, so `1Y` is driven LOW |
| **74HC08 unpowered or removed, or the TTL cable to the driver is broken** | **(c) only** |

Each pull-down costs its upstream push-pull output 0.5 mA — negligible against
the 74HC08's ≥4 mA drive.

**Location is part of the specification.** A pull-down works by holding a node
LOW when nothing else drives it; placed at the far end of a broken wire it
protects nothing. (a) belongs at the translator input pins, (b) next to the
74HC123/74HC08 input pins, and (c) at the connector where the TTL cable leaves
for the laser driver.

With all three fitted, removing any single IC from its socket becomes a
**verifiable** physical laser lockout — the TTL pin held LOW by a resistor, not
by trust — which is what makes it safe to work on the wiring downstream.

> **Why `/OE` is tied LOW instead of being driven by GPIO 18.** An SN74AHCT126N
> (active-HIGH `OE`, otherwise pin-identical) allows a "self-gated" variant in
> which GPIO 18 drives both `1A` and `1OE`, so the output is Hi-Z whenever the
> fire line is LOW. That variant is *not* used here. It adds no guarantee — Hi-Z
> is only safe because of pull-down (b), which is fitted either way — and it
> costs the actively-driven LOW: the **normal idle state** of the laser TTL line
> becomes a resistive pull rather than a push-pull LOW, on a high-impedance node
> running beside a switching laser driver. With `/OE` tied LOW the idle state is
> driven LOW *and* has (b) as a backstop. The '125 and '126 are
> indistinguishable on an assembled board, so fit only `AHCT125` and read the
> marking (§9.1).

### 9.4 Verification before either laser is connected

- [ ] Every fitted device is marked `AHCT`, not `AHC` or `HC`.
- [ ] Scope MOSI, SCLK, CE0, CE1 **at the MCP4922 pins** at the configured
      `spi_speed_hz`: valid HIGH/LOW levels, setup/hold met against the MCP4922
      timing spec. For breadboard bring-up, lower `spi_speed_hz` (e.g. 1 MHz) —
      one control cycle is four 16-bit transfers, so the clock is never the
      bottleneck, and 20 MHz edges on jumper wire will ring.
- [ ] Both `/CS` lines measure HIGH (DACs deselected) from the instant the 5 V
      rail comes up, through Pi boot, and before the application runs — this is
      what the CE0/CE1 pull-ups of §8 exist for.
- [ ] GPIO 18 held LOW → 74HC08 input measures LOW.
- [ ] Pull package #2 from its socket (or unpower it) → 74HC08 input stays LOW.
      **This tests pull-down (b), and nothing else does.**
- [ ] Disconnect the GPIO 18 wire at the translator input → same result. This
      tests pull-down (a).
- [ ] Pull the SN74HC08N from its socket → the voltage at the laser-driver TTL
      connector stays LOW. **This tests pull-down (c), and nothing else does.**
- [ ] Complete the §11a one-shot tests, including the pin-14 timing check.

The generic four-channel IIC/I2C module in `HARDWARE_INVENTORY.md` is
**excluded** from both paths. Those modules are open-drain with resistive
pull-ups intended for bidirectional 100–400 kHz I2C: RC-limited rise times are
orders of magnitude too slow for a 50 ns bit period at 20 MHz, and four channels
cannot cover five signals. It is retained for future I2C peripherals, not for
this build.

---

## 10. Galvo Driver Connections

The galvo driver board accepts a **differential analog input** for each axis. The two MCP4922 channels per axis drive the positive and negative inputs.

> **No-go until the driver's own manual confirms three things.** This is the
> assumption with the widest blast radius in the whole design: §14,
> `CoordinateMapper`, `DifferentialGalvoDriver` and the entire DAC choice rest on
> it, and "±5 V analog input at 0.33 V/°" in a seller listing establishes none of
> it. Because the two DAC channels are complementary unipolar 0–5 V outputs, the
> pair presents a **2.5 V common-mode voltage** that never goes away — it is not a
> transient, it is the operating point.
>
> 1. `IN+` / `IN−` are a genuine **differential** pair (a difference amplifier),
>    not two single-ended inputs summed, and not one input with the other intended
>    to be grounded.
> 2. The permitted **input common-mode range includes 2.5 V**. A driver on ±15 V
>    rails with a 1:1 difference amplifier will almost certainly accept it, but
>    "almost certainly" is not the standard applied anywhere else in this document.
> 3. The **±5 V figure is a differential rating**, not ±5 V per input measured to
>    ground. If it is the latter, the usable differential swing is different from
>    the one §14 assumes and every commanded angle is wrong by a scale factor.
>
> If any of the three is not as assumed, do not "try it and see" — the failure is
> a silent scale or offset error in where the beam points, which is exactly the
> class of fault a Class 4 system cannot absorb. Recheck it empirically during
> calibration too: command a known angle and measure the actual deflection before
> trusting the mapping (`CALIBRATION.md`).

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

Power path:

```
12 V + (K1-switched, §3) ─┬─► [ARM switch] ─► [DOOR INTERLOCK NC, §6a] ─► Laser driver +VIN
                          │
                          └─► Laser cooling fan   ← upstream of the arm switch, see below
12 V − ──────────────────────────────────────────────────────────────────► Laser driver GND
Laser driver GND ────────────────────────────────────────────────────────► Common GND
```

Control path (three fail-LOW pull-downs, one per node — §9.3):

```
GPIO 18 ──┬──► SN74AHCT125N #2 1A          [10 kΩ ──► GND]   (a) at the translator input
          │
          └──► 1Y ──┬──► 74HC123 1B/1CLR   [10 kΩ ──► GND]   (b) at the '123 / '08 inputs
                    └──► 74HC08 1A
                            │
                 74HC123 1Q ┘
                            │
                 74HC08 1Y ─┴──► verified   [10 kΩ ──► GND]   (c) at the driver connector
                                 laser-driver TTL-switch pin
```

### Safety notes

- The laser driver receives **no power** until K1 is energised (§3), the arm switch is ON, and the enclosure door is closed (§6a). These are the hardware interlocks.
- The TTL input is the **software trigger**, but it passes through the pulse-duration backstop of §11a — the laser fires only when GPIO 18 is HIGH, the arm switch is ON, the door is closed, the E-Stop is released, the one-shot has not timed out, and all software safety gates are satisfied.
- **The driver's TTL polarity has not been established.** Confirm from the
  module's own documentation that the input is **active HIGH** before connecting
  it. Every fail-LOW measure in §9.3 and §11a assumes LOW = off. On an
  active-LOW input the entire chain is inverted and each of those pull-downs
  becomes a *fire* command — the pull-downs would have to become pull-ups and the
  AND gate would have to become a different gate. Do not infer polarity from a
  seller listing, and do not determine it experimentally with the Class 4 module
  connected.
- **Power the cooling fan upstream of the arm switch**, so it runs whenever the
  12 V branch is live. Wired downstream, disarming stops the fan instantly on a
  diode that was dissipating 2.5 W a moment earlier — thermal stress is worst
  right after a firing sequence, which is exactly when an operator reaches for
  the arm switch. Verify the fan's current is inside the LRS-50-12 budget
  alongside the driver.
- Do not energize the working module without its forced-air cooling operating.
- Do not use the advertised PWM power-control mode. The firing path must remain
  one sustained level: repeated PWM edges can retrigger the 74HC123 and defeat
  its duration cap.
- Keep the TTL line short and away from the laser power cable to reduce noise.
- The laser module chassis must be bonded to protective earth (§13).

---

## 11a. Laser TTL Pulse-Duration Backstop (74HC123)

Every *software* mechanism that ends a laser pulse (max-pulse enforcement,
watchdog, E-stop poll) runs on the **control thread**. If that thread hangs with
GPIO 18 stuck HIGH, no software turns the laser off. This one-shot is the
**independent hardware duration backstop** — it is not on the control thread and
needs neither software nor an operator to act. Its input comes from
SN74AHCT125N #2 (§9), which is what makes a 3.3 V GPIO able to drive these 5 V
HC-family inputs at all. Its bound is the measured
assembled pulse width, not the nominal R/C calculation. See AGENTS.md §4.1.

### Circuit

The laser TTL is gated by `translated GPIO18 AND one-shot-Q`. The one-shot is
triggered by the translated GPIO 18 rising edge; when it times out, the AND gate
force-drives the laser TTL LOW even if GPIO 18 is still HIGH.

```
SN74AHCT125N #2 1Y (pin 3, §9) ─────┬───────────────► 74HC08 gate 1 input 1A (pin 1)
                                     ├───────────────► 74HC123 1B (pin 2, +edge trigger)
                                     ├───────────────► 74HC123 1CLR/1RD (pin 3)
                                     └── 10 kΩ ──────► GND  (fail-LOW on Hi-Z, §9.3 (b))
74HC123 1A (pin 1) ─────────────────────────────────► GND (required for 1B rising trigger)
74HC123 active-HIGH 1Q (pin 13) ────────────────────► 74HC08 gate 1 input 1B (pin 2)
74HC08 gate 1 output 1Y (pin 3) ────┬───────────────► verified laser-driver TTL switch pin
                                    └── 10 kΩ ──────► GND  (fail-LOW at the laser-driver
                                                            connector, §9.3 (c))

Timing network (channel 1) — the pin numbers below are load-bearing, see the
warning that follows this block:
    74HC123  1Cext (pin 15) ──── 1 µF / 50 V monolithic ceramic ──── 1Rext/Cext (pin 14)
    74HC123  1Rext/Cext (pin 14) ── 220 kΩ ──► +5 V
    74HC123 VCC (pin 16) ──► +5 V ; GND (pin 8) ──► common GND
    100 nF ceramic directly across 74HC123 pins 16↔8
    Unused channel 2: hold 2CLR/2RD (pin 11) LOW; tie 2A/2B (pins 9/10) to defined levels;
                      leave 2Cext (pin 6) and 2Rext/Cext (pin 7) open

AND gate:
    SN74HC08N VCC (pin 14) ──► +5 V ; GND (pin 7) ──► common GND
    100 nF ceramic directly across SN74HC08N pins 14↔7
    Tie every unused AND-gate input to a defined HIGH or LOW; leave unused outputs open
```

> **The two channels are mirrored — do not carry channel 2's pin order into
> channel 1.** On the standard 74HC123 pinout channel 2 is `6 = 2Cext,
> 7 = 2Rext/Cext`, while channel 1 is `14 = 1Rext/Cext, 15 = 1Cext`. The order
> reverses. A previous revision of this document had 14 and 15 swapped, which
> puts the 220 kΩ on the discharge node instead of the timing node: the one-shot
> does not time as designed, and the mandatory scope measurement below then
> corresponds to nothing in the design. **The 220 kΩ goes to pin 14.** Check it
> against the datasheet of the device actually in your hand before powering the
> board.

Pin 13 is the channel-1 **active-HIGH Q** output on the standard 74HC123 pinout;
pin 4 is its active-LOW complement and must not be used for the AND gate. Verify
the full marking on the on-hand IC against its manufacturer's datasheet before
wiring. Holding 1A LOW is required for a rising edge on 1B to trigger the
monostable. Tying 1CLR/1RD to the translated fire level holds the channel reset
while idle and allows the rising fire transition to arm/trigger it; this exact
behaviour must be included in the scope tests below.

The `1Y → 74HC08` output pull-down (c) is what holds the **laser driver's own**
TTL pin LOW when the 74HC08 is unpowered, removed from its socket, or its output
cable is broken. Pull-downs (a) and (b) protect the translator; neither reaches
past the AND gate. Size (c) once the laser driver's TTL input has actually been
characterised (§11): 10 kΩ is the starting value, but if that input carries an
internal pull-up, (c) must be low enough to hold it below the driver's `V_IL`
with the 74HC08 absent, while still letting the 74HC08 pull it HIGH.

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
- [ ] **Fail-LOW on a dead translator:** pull SN74AHCT125N #2 from its socket (or
  unpower it) and confirm the laser TTL stays LOW. This proves pull-down (b) of
  §9.3 — the AND gate does not cover this case, because a floating input can read
  HIGH.
- [ ] **Fail-LOW on a dead AND gate:** pull the SN74HC08N from its socket (or
  unpower it) and confirm the voltage at the **laser-driver TTL connector** stays
  LOW. This proves pull-down (c). Nothing upstream of the AND gate covers this
  case: (a) and (b) are both on the translator side.
- [ ] **Timing pins verified against the datasheet:** 220 kΩ on **pin 14**
  (1Rext/Cext), capacitor between pins 15 and 14. Wired to pin 15 instead, the
  measured period is meaningless.
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
- The bipolar supply's **0 V / COM** conductor is the reference for both the +15 V
  and −15 V rails and is the galvo driver's signal reference. It must be part of
  the common signal reference above, not left floating — the differential inputs
  of §10 measure against it.
- Use shielded USB 3 cables for the cameras and route them away from the laser power cable.
- The laser module chassis, the galvo head/driver chassis, and all metal
  enclosure parts must be bonded to protective earth (PE), not just logic GND.
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

**Hazardous power comes up last, and the hardware — not the operator — enforces
that.** Because K1 latches (§3), closing the mains isolator does *not* energise
the galvo ±15 V or the laser 12 V. There is no ordering an operator can forget:
the supplies are off until someone presses START, and START is the last step.

### Power-up

1. Verify the arm switch is **OFF**, the enclosure door is closed, and the E-stop
   is released.
2. Close the mains isolator. **K1 stays de-energised — ±15 V and 12 V remain OFF.**
3. The Pi boots from its own USB-C supply. The 5 V logic rail comes up with it, so
   the §9.3 pull-downs and the AND gate are defined from the first instant.
4. With the hazardous supplies still off, confirm with a meter:
   - laser-driver TTL connector: **LOW**
   - all four DAC outputs: **≈ 2.5 V** (against the measured rail, §8)
   - GPIO 25 ≈ 2.53 V, GPIO 24 ≈ 0 V
5. Start the application. Verify both camera streams open and that
   `[CONFIG] Aborting` does not appear.
6. Put on safety eyewear. Confirm the enclosure is closed and the beam path is
   contained.
7. **Press START.** K1 pulls in and the ±15 V and 12 V supplies energise; the
   galvos take up their commanded centre.
8. Turn the arm switch **ON** only when ready to run.

Steps 4 and 5 happen before step 7 by design: the logic has demonstrated a
centred galvo pair and a LOW laser TTL *before* anything hazardous has power.

### Power-down

1. Turn the arm switch **OFF**.
2. Wait 10 seconds — the firing cooldown, and time for the fan to pull heat out of
   the module (the fan runs while the 12 V branch is live, §11).
3. Press **STOP**, or the E-stop. K1 drops out; ±15 V and 12 V are removed.
4. Trigger software shutdown (`sudo shutdown now`).
5. After the Pi halts, open the mains isolator.

### Emergency

Press the mushroom E-Stop at any time. K1 drops out, removing ±15 V and 12 V: the
laser driver cannot emit and the galvos cannot move. GPIO 25 goes LOW and the
software enters `SAFE_HALT`. The Pi keeps running on its own supply, so the log of
what happened survives.

**Releasing the mushroom does nothing.** K1's hold path is broken and only a
deliberate START press restores power (§3).

`SAFE_HALT` is terminal, so recovery also requires restarting the software. **Put
the arm switch OFF before you do.** The software cannot know an E-stop occurred
before it started; restarting with the mushroom released and the arm switch still
ON is exactly the automatic-restart path §3 exists to eliminate, and it would take
the state machine from `INIT` to `ARMED` with no deliberate arming action.

---

## 16. Bench Testing Without the Laser

For initial bring-up, leave the laser **disconnected or switched off** and use a multimeter or oscilloscope on the DAC outputs.

1. Build the Pi, DACs, the SN74AHCT125N translation of §9, and the galvo-driver
   electronics. Verify all four DAC outputs sit at ≈2.5 V before anything else:
   the MCP4922 latches its last written value, so you can run the binary, let it
   exit, and measure at leisure.
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

### Bench shortcuts worth knowing

- **`/LDAC` first.** If the DAC outputs never move while the SPI traffic looks
  perfect on a scope, `/LDAC` (pin 8) is floating instead of grounded (§8). This
  failure is silent: `MCP4922::write()` returns success either way.
- **The MCP4922 latches its last written value**, so you can run the binary with
  unconfigured cameras, let it fail out, and measure the 2.5 V mid-scale at
  leisure.
- **Jumper the sense inputs during bench runs.** A floating GPIO 25 reads as
  *E-stop pressed*, and the resulting `SAFE_HALT` is terminal — the run dies
  immediately for reasons that look like a software fault. Tie GPIO 25 to 3V3
  through 10 kΩ and GPIO 24 to GND for a clean disarmed state until the real sense
  networks of §5 and §6 exist.
- **Lower `spi_speed_hz` to about 1 MHz for breadboard work.** One control cycle
  is four 16-bit transfers, so the clock is never the bottleneck, and 20 MHz edges
  on jumper wire will ring (§9.4).

---

## 17. Build & Bring-Up Order

The section numbering above is a **reading** order. It is not a build order, and
following it as one would put live mains in the work area at step 3 — while the
circuits are still on a breadboard and before the mushroom's contacts have been
verified.

Build in **risk order** instead. Each stage has a gate; do not start the next
stage until the gate passes.

**Stages 0 to 5 require no mains at all.** The 5 V logic rail comes from the Pi
header (§4), so the whole logic and interlock chain can be built and verified on
SELV. The mains package may be *designed and fabricated* in parallel at any time —
it is a separate work package for a qualified person — but it must not be
*energised in the work area* until stage 6.

| Stage | Work | Gate |
|-------|------|------|
| **0** | **Freeze and inspect.** Reconcile every drawing against this document. Meter every resistor (3.3 kΩ vs 3.3 Ω). Read the marking on every IC (`AHCT`, not `AHC`/`HC`). Ring out the mushroom and lever-switch contacts. Obtain the galvo driver manual (§10). | Every part identified by measurement rather than by the bag it came in; §10's three questions answered from the manual |
| **1** | **Enclosure mechanics.** Case, beam dump, door-interlock switch mounting, PE bonding plan. No electronics. | Door interlock actuates on door movement and cannot be defeated without a tool (§6a) |
| **2** | **Pi alone.** USB-C only. Toggle GPIO 18/24/25, watch edges with `gpiomon`. No 5 V rail, nothing else connected. | Every pin behaves; `gpiomon` timestamps are sane |
| **3** | **DAC island.** 5 V logic rail, AHCT125 #1, both MCP4922s **including `/LDAC` to GND and `/SHDN` to +5 V**, CE0/CE1 pull-ups. | All four DAC outputs ≈ 2.5 V; the 5 V rail measured at the DAC pin under load and written into `dac_reference_voltage` (§8) |
| **4** | **Pulse-duration backstop.** AHCT125 #2, 74HC123 with the **220 kΩ on pin 14**, SN74HC08N, pull-downs (a)(b)(c). Scope and dummy load only — no laser of any class. | Every checkbox in §11a passes and **the measured period is recorded** |
| **5** | **Sense networks.** Arm and E-stop dividers, Zeners with the band at the junction, door-interlock contacts. Meter each node **with the GPIO wire disconnected**, then attach. | ≈ 0 V / 2.98 V on arm OFF/ON; ≈ 0 V / 2.53 V on E-stop pressed/released (§5, §6) |
| **6** | **Mains package.** Isolator, overcurrent protection, RCD, K1, START/STOP, terminal blocks, PE bonding. Designed, built and inspected by a qualified person. | Latching verified: E-stop drops K1; **releasing it does nothing**; only START restores. Both DC rails measured at 0 V with the E-stop pressed (§3) |
| **7** | **DC supplies, unloaded.** Energise ±15 V and 12 V with nothing connected. | +15 V, 0 V, −15 V and 12 V all within tolerance; COM bonded to common ground (§13) |
| **8** | **Galvo driver, no head.** Supplies, ground, and the analog inputs per the topology verified at stage 0. | Commanded DAC pairs produce the expected differential at `IN+`/`IN−` with no clipping or offset (§10, §14) |
| **9** | **Galvo head connected, no optical source.** | Mirrors centre on start-up and slew to commanded angles without hitting a mechanical limit |
| **10** | **Cameras and calibration.** By-path identification and labelling, stereo calibration, dry-run tracking. | `PRE_FLIGHT_CHECKLIST.md` §3 and §4 complete |
| **11** | **Verified low-power alignment source**, inside the **closed** interlocked enclosure. | `PRE_FLIGHT_CHECKLIST.md` §5 complete |
| **12** | **Class 4 module.** | Only after a final electrical, laser-safety and functional-safety review, with eyewear, beam dump and every no-go item in `HARDWARE_INVENTORY.md` closed |

Two stages are the ones most likely to be skipped under time pressure, and both
are the reason the rest of the document exists: **stage 4's recorded one-shot
period** is the only measurement that turns the pulse-duration backstop from a
schematic into a guarantee, and **stage 6's latching check** is the only one that
proves an E-stop release cannot restart the machine.
