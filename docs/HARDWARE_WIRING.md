# Hardware Wiring Guide — Mosquito Laser Killer

**WARNING:** This system controls a **Class 4 laser** (2.5 W, 450 nm). Class 4 lasers cause instantaneous, irreversible eye injury and can ignite materials. Do not apply power to the laser until all enclosures, beam dumps, interlocks, and OD 4+ safety eyewear are in place. Read `AGENTS.md` before modifying code or wiring.

This document is the single source of truth for the **intended** stereoscopic
laser-targeting wiring. Readings actually taken on the bench — expected value,
measured value and date — are logged in [`MEASUREMENTS.md`](MEASUREMENTS.md). It matches the source code in `src/hal/mcp4922.cpp`,
`src/hal/differential_galvo_driver.cpp`, and
`src/control/coordinate_mapper.cpp`. The parts physically reported on hand are
tracked separately in [`HARDWARE_INVENTORY.md`](HARDWARE_INVENTORY.md).

> **Verification no-go:** nothing in this build is verified. No switch contact
> has been ringed out, no IC marking read, no passive metered, and the 74HC123
> period has not been measured. The safety-contactor architecture of earlier
> revisions was **abandoned unbuilt**: §3 now specifies a 12 V interlock chain
> with no latch, and the residual risks that creates are listed there. Two
> unknowns are blocking
> on their own: the **galvo driver's differential input topology and
> common-mode range** (§10), which invalidates §14 and the coordinate mapping if
> it turns out wrong, and the **laser driver's TTL polarity** (§11), because on
> an active-LOW input every fail-LOW pull-down in §9.3 becomes a fire command.
> Do not apply power until every reconciliation item in
> `HARDWARE_INVENTORY.md` is closed.

---

## Table of Contents

1. [System Block Diagram](#1-system-block-diagram)
2. [Inventory-Aware Wiring Bill of Materials](#2-inventory-aware-wiring-bill-of-materials)
3. [Mains Power & the 12 V Interlock Chain](#3-mains-power--the-12-v-interlock-chain)
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
| [2 — mains power distribution](diagrams/2%20-%20mains-power-distribution.png) | **STALE** — drawn for the abandoned K1 contactor architecture. Redraw against §3, §4 |
| [3 — GPIO sense circuits](diagrams/3%20-%20gpio-sense-circuits.png) | **STALE** — the E-stop divider is now 12 V-sourced through a 10 kΩ. Redraw against §5, §6, §6a |
| [4 — SPI level translation](diagrams/4%20-%20spi-level-translation.png) | AHCT125 #1, both MCP4922s with full pin assignment, galvo inputs (§8, §9, §10) |
| [5 — laser TTL safety chain](diagrams/5%20-%20laser-ttl-safety-chain.png) | AHCT125 #2, the 74HC123 one-shot, the AND gate, all three pull-downs (§9.3, §11, §11a) |
| [6 — complete control-board wiring](diagrams/6%20-%20complete-control-board-wiring.png) ([editable SVG](diagrams/6%20-%20complete-control-board-wiring.svg)) | Consolidated Pi-header-to-module wiring, logic rails, sense networks, external connectors. **Its power map is stale** — redraw against §3 |

The drawings follow this document, not the other way round. Where they disagree,
this document is correct and the drawing is stale — fix the drawing, and never
build from a drawing that contradicts it.

```
Wall socket ─► ±15 VDC supply ─────────────────────► Galvo driver ─► X/Y galvo head
  (building RCD/OCP is the                             (unswitched — always live)
   upstream protection, §3)
Wall socket ─► LRS-50-12 ─► 12 VDC ─┬─► cooling fan (unswitched, §11)
                                    │
                                    └─► [E-STOP NC] ─┬─► [ARM] ─► [DOOR NC] ─► Laser driver ─► 2.5 W 450 nm
                                                     │                    │
                                        GPIO 25 ◄────┘       GPIO 24 ◄────┘
                                        (E-stop only)        (all three)

  Three contacts in one conductor. No contactor, no latch: releasing the
  mushroom restores the driver's 12 V, and only SAFE_HALT + GPIO 18 LOW +
  the §9.3 pull-downs keep the beam off (§3).

Separate USB-C 5 V ─► Raspberry Pi 5.  Never switched: the log, both GPIO sense
                                       circuits and the 5 V logic rail that holds
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
| Galvo PSU | Regulated bipolar ±15 VDC supply sized for both driver channels | 1 | On hand: **the galvo kit's own supply, with its mating cables** — open-frame board marked `HT-15V30W V2.0-181206`, 30 W total, mains and DC both on **pin headers rather than terminal blocks**, PCB exposed. Rail voltages, per-rail current and the `COM` pin position all unrecorded; `COM` is not yet bonded to the logic ground the DAC pair is referenced to (§13) |
| Mains cords | Two factory-moulded power cords, one per supply. No plug assembly, no switched mains, no project-side isolator or OCP — the wall socket's RCD and breaker are the upstream protection (§3) | 2 | Not yet fitted |
| Door interlock switch | **1** NC contact with **positive/direct opening action** (IEC 60947-5-1 Annex K), rated for the driver current at 12 V DC, tool-required actuator (§6a) | 1 | On hand; contact arrangement, DC rating and positive-opening marking not yet confirmed |
| X-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Reported on hand |
| Y-axis DAC | MCP4922 DIP-14, 12-bit dual DAC | 1 | Reported on hand |
| SPI translation | SN74AHCT125N, PDIP-14 quad bus buffer — push-pull, non-inverting, TTL input thresholds; carries MOSI, SCLK, CE0, CE1 | 1 package (4 of 4 ch) | On hand, untraceable brand; **marking certifies nothing.** Qualify by the §9.4 threshold test |
| Laser translation | SN74AHCT125N, a **second physically separate** package, 1 channel used, with the two fail-LOW pull-downs of §9.3 | 1 package (1 of 4 ch) | On hand, untraceable brand; **qualify by the §9.4 threshold test.** Package #1 is fully consumed by SPI, so this is a second device, not a spare channel |
| Monostable | 74HC123, DIP-16 dual retriggerable one-shot | 1 | Reported on hand; manufacturer/order code unverified |
| AND gate | SN74HC08N, DIP-14 quad 2-input AND | 1 | Reported on hand |
| Timing resistor | 220 kΩ, 1/2 W | 1 | Reported on hand |
| Timing capacitor | 1 µF, 50 V monolithic ceramic | 1 | Reported on hand; tolerance/effective capacitance unverified |
| Logic/DAC decoupling | 100 nF ceramic directly across the supply pins of each MCP4922, SN74AHCT125N, 74HC123, and SN74HC08 | 6 minimum | Value reported; available count unverified |
| Rail bulk decoupling | 10 µF on the 5 V rail at the logic board | 1 | On hand |
| Arm switch | Lever switch with contacts rated for the measured 12 VDC laser-driver load | 1 | Reported on hand; topology/rating unverified |
| E-stop | Mushroom actuator, **1 NC** in the 12 V laser feed (§3). Needs a DC rating for the driver current and the Annex K direct-opening symbol. The shipped NO is left unwired | 1 | On hand: 1 NO (terminals 3–4) + 1 NC (terminals 1–2), generic unmarked blocks. DC rating and Annex K status **unconfirmed** |
| Arm series resistor | 10 kΩ, 1/2 W | 1 | Reported on hand |
| Laser-path fail-LOW pull-downs | 10 kΩ, 1/2 W — **three**, one per node: (a) translator input, (b) '123/'08 inputs, (c) laser-driver connector (§9.3) | 3 | Value reported on hand; count unverified |
| Chip-select pull-ups | 10 kΩ, 1/2 W to **3.3 V** on the CE0 and CE1 translator inputs (§8) | 2 | Value reported on hand; count unverified |
| E-stop series resistor | 10 kΩ, 1/2 W — the sense is now 12 V-sourced, so it uses the arm circuit's divider (§6) | 1 | On hand; value not yet metered |
| Sense pull-downs | 3 kΩ, 1/2 W | 2 | On hand; value not yet metered |
| Zener clamps | BZX55C3V3, DO-35, 0.5 W — cathode at the sense junction, anode to GND | 2 | Part type reported; count unverified |
| Sense/debounce capacitors | 100 nF, 50 V monolithic ceramic | 2 | Value reported; count unverified |
| Distribution connectors | WAGO 221-413, 3-conductor, max 4 mm² — splicing connectors, **not** terminal blocks and not a barrier between circuits | as required | Reported on hand; count unverified |
| Enclosure | Laser-safe interlocked case with beam dump | 1 | On hand; rating against 450 nm at 2.5 W not yet recorded |
| Safety eyewear | Correctly rated for 450 nm and the documented exposure analysis | 1 per person | On hand; OD figure and marking not yet recorded |

**Running totals for the parts that appear in more than one circuit.** These are
the counts to shop against; the rows above are the counts per role.

| Value | Total | Where |
|-------|-------|-------|
| 10 kΩ, 1/2 W | **7** | 1 arm series + **1 E-stop series** + 3 laser-path fail-LOW (a)(b)(c) + 2 chip-select pull-ups |
| 100 nF, 50 V ceramic | **8** | 6 IC decoupling (2× MCP4922, 2× AHCT125, 74HC123, SN74HC08) + 2 sense debounce |
| 3 kΩ, 1/2 W | **2** | arm sense divider + E-stop sense pull-down |
| BZX55C3V3 | **2** | one clamp per sense junction |
| 10 µF | **1** | 5 V rail bulk |
| 220 kΩ, 1/2 W | **1** | 74HC123 timing, to **pin 15** (§11a) |
| 1 µF, 50 V ceramic | **1** | 74HC123 timing, pins 15↔14 (§11a) |

Meter every resistor before installation and fit it against the value in the
table above, not against the bag it came in. A wrong value in either sense
network holds that input LOW and makes the interlock it reports inoperable, and
nothing about the assembled board makes that visible.

The same hazard applies to the logic family: `AHCT`, `AHC`, and `HC` share the
same pinout and differ by one letter in the marking, but only `AHCT` accepts a
3.3 V input at a 5 V supply (§9.1). Read the marking on every fitted device — and, for the two `AHCT125`s, measure the input threshold too (§9.4): the devices on hand have no manufacturer traceability, so their marking is not evidence.

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

## 3. Mains Power & the 12 V Interlock Chain

The E-stop must remove the beam. In this build it does that by **breaking the
laser driver's 12 V supply**, in series with the arm switch and the door
interlock. There is no switched mains, no contactor and no START button.

> **This is the second revision of this section. Both earlier designs are
> recorded so the reasoning is not lost and the mistakes are not reintroduced.**
>
> 1. **Mains through the mushroom contact** — rejected. The LRS-50-12 specifies a
>    **45 A cold-start inrush at 230 VAC**. A pilot-duty contact block subjected
>    to that **welds**, leaving an E-stop that looks and feels normal and
>    disconnects nothing. That objection is fully satisfied by the present
>    design: the mushroom switches a 12 V DC branch, not mains.
> 2. **A latching contactor K1 on the mains**, with the mushroom breaking only
>    its coil circuit — designed, then **abandoned unbuilt.** No contactor was
>    ever obtained, and this build is done by one person with no qualified
>    electrician available, which makes a hand-built mains control circuit the
>    highest-risk work in the project. Specifying an architecture that will not
>    be built is worse than specifying a weaker one that will: a guard that
>    exists on the bench beats a guard that exists in a document.
>
> **What K1 bought and this does not is the latch.** That gap is real, it is
> stated in full below, and it has a compensating control.

### Architecture

```
wall socket ─► LRS-50-12 ─► 12 V+ ─►[E-STOP NC]─┬─►[ARM lever]─►[DOOR NC]─┬─► laser driver +VIN
                                                │                          │
                                                │                          └─► 10 kΩ ─► GPIO 24 (§5)
                                                └─► 10 kΩ ─► GPIO 25 (§6)
                           12 V− ─────────────────────────────────────────────► laser driver GND

wall socket ─► bipolar ±15 V supply ─► galvo driver        (unswitched, always live)
separate USB-C ─► Raspberry Pi 5 ─► 5 V logic rail         (unswitched, always live)
```

Three contacts in series on one conductor. Any one of them opening removes
`+VIN` from the laser driver, which is the only thing that can produce a beam.

| Contact | Opens when | Effect |
|---------|-----------|--------|
| E-stop NC | mushroom pressed | laser driver dead; GPIO 25 LOW → `SAFE_HALT` |
| Arm lever | operator disarms | laser driver dead; GPIO 24 LOW → disarm to `IDLE` |
| Door NC | enclosure opened | laser driver dead; GPIO 24 LOW → disarm to `IDLE` |

**The two sense taps sit at different points in the chain, and that is the only
reason the software can tell an emergency from a disarm.** GPIO 25 is tapped
immediately after the mushroom, so it reports the mushroom alone. GPIO 24 is
tapped after all three, so it reports "the driver actually has power available".
A pressed mushroom takes both LOW, and the E-stop path wins because
`control_step()` polls it before the arm logic.

### Upstream protection is the building's, deliberately

**Nothing in this project switches, fuses or modifies mains.** Both supplies are
plug-connected to an existing wall socket, and that socket's RCD and overcurrent
protection are the upstream protection an earlier revision listed as project
parts to buy and install.

This is the single largest risk reduction available in this build. It is done by
one person with no electrician; the correct response is to drive the amount of
mains work towards zero, not to write "have it inspected by a qualified person"
next to work that will not be inspected. What remains is three screw
terminations per supply.

- Use a **factory-moulded power cord**. Do not assemble a plug.
- Brown → `L`, blue → `N`, **green/yellow → the supply's earth terminal**, and
  from there to the enclosure, the laser chassis and the galvo chassis (§13).
  PE is never switched and never fused.
- Keep each supply's terminal cover fitted.
- Worth ten euros: a panel **IEC C14 inlet with integrated fuse and switch**, so
  the only bare mains conductors are a short run inside a closed box.

### What this architecture does not do

Every item below is a real reduction against the contactor design. They are
listed so nobody has to rediscover them, and so the residual risk is owned
rather than forgotten.

1. **Releasing the mushroom restores the laser driver's 12 V.** There is no
   latch. What prevents a beam on release is logic state, not the absence of
   power: `SAFE_HALT` is terminal, GPIO 18 is LOW, and the three fail-LOW
   pull-downs of §9.3 hold the TTL line down. The case this does *not* cover is
   a control thread that hung with GPIO 18 **HIGH** — on release the laser then
   emits for one 74HC123 one-shot period before the backstop cuts it. **This is
   why the measured one-shot period (§11a) is a safety number in this build**,
   not merely a design check, and why a period well above 100 ms is worth
   shortening.
2. **`SAFE_HALT` is terminal, so recovery means restarting the software** — and
   restarting with the arm lever still ON walks the state machine
   `INIT → IDLE → ARMED → TRACKING → FIRING` with no deliberate action anywhere
   in the sequence. **Required compensating control:** `ArmSwitch` must observe
   a LOW→HIGH transition before it ever reports armed, so a process that starts
   with the lever already up stays disarmed until the operator cycles it. That
   restores in software the deliberate-start property K1 provided in hardware.
   It is **not yet implemented** and is an open item in
   `HARDWARE_INVENTORY.md`.
3. **Mains stays live inside the enclosure at all times.** The mushroom is not
   an isolator and an E-stop is not a lock-out. To work inside, **unplug both
   supplies.**
4. **The galvo ±15 V is never removed.** Mirrors can still move during an
   E-stop. With the laser unpowered that is not a beam hazard, but the E-stop no
   longer produces the "both DC rails at 0 V" state the previous commissioning
   test looked for, and §15 and §17 have been changed accordingly.
5. **One contact carries the entire hardware interlock.** If the mushroom's NC
   welds closed, neither the hardware nor the software sees the E-stop — GPIO 25
   is derived from the same conductor. The defence is **positive/direct opening
   action** (IEC 60947-5-1 Annex K), which forces the contact open mechanically
   instead of relying on a spring. The block fitted is a generic unmarked part
   whose Annex K status is **unconfirmed**. Open no-go item.

### The mushroom's contact duty

The NC block now switches the laser driver's inrush and running current at
12 V DC. Two things must be confirmed before the Class 4 module is connected:

- **A DC rating, not an AC one.** Direct current has no zero crossing, so a
  block marked `AC-15 240 V 3 A` can be rated for far less under DC-13. Measure
  the driver's actual current draw and check it against the block's DC rating.
- **Positive opening (Annex K)**, per item 5 above.

The same duty applies to the arm lever and the door contact, which sit in the
same conductor.

### Wiring practice

- The mushroom's second contact is a **NO** and is **left unwired.** It cannot
  carry any part of this function: a NO reaches its safe state by *closing*, so
  a broken wire leaves it reading "healthy". No wiring arrangement fixes that —
  it is a property of the contact, not of the circuit.
- 12 V and SELV wiring segregated from the mains tails, strain-relieved, inside
  the enclosure.
- One WAGO 221-413 per net, nothing shared. They are splicing connectors, not a
  barrier between circuits.

---

## 4. DC Power Distribution

**Nothing is switched by the E-stop except the laser driver's own feed.** All
three supplies are plug-connected and stay live; the three series contacts of §3
remove `+VIN` from the laser driver alone.

| Supply | Output | Removed by an E-stop? | Feeds | Notes |
|--------|--------|-----------------------|-------|-------|
| Bipolar galvo supply | **+15 V, 0 V/COM, −15 V** — three conductors, not "±15 V" as one wire | **No** | Galvo driver board | Unswitched. Mirrors can still move during an E-stop; with the laser unpowered that is not a beam hazard (§3). On hand but not yet measured. COM is the reference for both rails and bonds to the common ground (§13) |
| Mean Well LRS-50-12 | 12 VDC | **The supply, no. Its laser-driver branch, yes** | Laser cooling fan directly; laser driver via E-stop + arm lever + door contact | The three contacts sit in the `12 V+` conductor between the supply and the driver. Do not use this single-output supply as a substitute for the bipolar ±15 VDC |
| Pi USB-C supply | 5 VDC | **No** | Raspberry Pi 5 | A separate supply, so the log, the GPIO sense circuits and the 5 V logic rail all survive an E-stop |
| 5 V logic rail | 5 VDC, from Pi header pins 2/4 | **No** | Both SN74AHCT125N, both MCP4922 VDD/Vref, 74HC123, SN74HC08 | Derived from the Pi, so it tracks the Pi and survives an E-stop |

Two consequences of that last row are load-bearing:

- **The 5 V logic rail survives an E-stop.** The three fail-LOW pull-downs of
  §9.3, the AND gate and the one-shot all stay powered and defined while the
  laser driver is dead. An E-stop removes the hazard without leaving the control
  chain in an undefined state — which matters more here than it did under the
  contactor design, because releasing the mushroom restores the driver's 12 V
  and the TTL line must already be held LOW when it does (§3).
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
12 V + via [E-STOP NC, §3] ──► [ARM SPST] ──► [DOOR INTERLOCK NC, §6a] ──┐
                                                                          │
                                                       Laser driver +VIN ◄─┤
                                                                          │
                              ┌───────────────────── 10 kΩ ◄──────────────┘
                              │
              GPIO 24 ◄───────┼── junction ──┬── 3 kΩ ───────► GND
            (RPi pin 18)      │              ├── 100 nF ─────► GND
                              └──────────────┴── BZX55C3V3 ──► GND
                                                 banded (cathode) end at the
                                                 junction, anode at GND
```

### Component notes

- **10 kΩ**: current-limiting series resistor from the 12 V arm signal.
- **3 kΩ**: lower leg of the voltage divider, **connected to the same junction**
  as the 10 kΩ, the 100 nF, the Zener and the GPIO. GPIO 24 sees
  `12 V × 3 kΩ / (10 kΩ + 3 kΩ) ≈ 2.77 V` — a valid 3.3 V logic HIGH.
  Without it there is no divider: the GPIO pin sees 12 V through 10 kΩ with the
  Zener as the only thing standing between the rail and a 3.3 V input, drawing
  about 0.9 mA through it continuously. **It still reads HIGH**, which is what
  makes the omission dangerous — the circuit appears to work until the Zener
  fails.
- **Meter the lower leg before soldering it.** An undersized value drops the
  junction below `V_IH`, so the Pi could never observe an armed state. Taking
  `V_IH` at a conservative 2.0 V, the floor for this leg is ≈ 2 kΩ. At 3 kΩ the
  worst-case stack — both resistors at the far end of ±5% *and* the 12 V rail
  sagging to 11 V — still lands at ≈ 2.35 V, so the margin is not consumed by
  tolerance. Anything at or below 2 kΩ is a no-go, whatever the bin says.
- **The value is 3 kΩ, not 3.3 kΩ, and that is deliberate.** Earlier revisions of
  this document specified 3.3 kΩ; the stock actually on hand is 3 kΩ, the
  computed node voltages above are the ones the board produces, and every
  expected reading in §6, §15 and §17 was recomputed to match. Do not "restore"
  3.3 kΩ without also restoring 2.98 V / 2.53 V everywhere they appear.
- **100 nF**: filters switch bounce and high-frequency noise.
- **BZX55C3V3**: **cathode (the banded end) at the junction, anode at GND.** It
  clamps transients above ~3.3 V. Fitted backwards it is a forward-biased diode
  that pins the junction near 0.7 V, so the Pi can never read armed — fail-safe,
  but silent and baffling. Confirm the band before soldering; a diode-test on a
  multimeter reads the polarity in seconds.
- The arm switch must be rated for the laser driver current (typically < 1 A at
  12 V).
- **The sense tap is downstream of all three contacts, deliberately** — E-stop,
  lever and door. GPIO 24
  then means "the laser driver actually has power available," not merely "the
  lever is up": opening the enclosure door reads as a *disarm*, which the
  existing `ArmSwitch` → `FiringController::set_armed(false)` path already handles
  by clearing targets and rejecting fire. The door interlock becomes visible to
  the software through a path that already exists, with no code change and no
  additional GPIO.

---

## 6. E-Stop GPIO Circuit

The mushroom's single NC contact does the hardware work of §3: it breaks the
laser driver's 12 V. The software E-stop on GPIO 25 is derived from **the same
conductor**, tapped immediately after that contact and before the arm lever.

```
12 V+ ──► [E-Stop NC] ──┬──► [ARM lever] ──► [DOOR NC] ──► laser driver +VIN
                        │
                        └──► 10 kΩ ──┐
                                     │
            GPIO 25 ◄──── junction ──┼── 3 kΩ ───────► GND
          (RPi pin 22)         │     ├── 100 nF ─────► GND
                               └─────┴── BZX55C3V3 ──► GND
                                         banded (cathode) end at the
                                         junction, anode at GND
```

`12 V × 3 kΩ / (10 kΩ + 3 kΩ) ≈ 2.77 V` — a valid 3.3 V logic HIGH. The network
is now **identical to the arm circuit of §5**: same 10 kΩ series leg, same 3 kΩ
lower leg, same debounce and clamp, same expected voltage. The 1 kΩ series
resistor of the earlier 3.3 V-sourced design is no longer used anywhere in this
project.

All five legs — the 10 kΩ from the tap, the 3 kΩ, the 100 nF, the Zener and the
wire to GPIO 25 — meet at **one** junction. The 3 kΩ is what pulls the pin LOW
when the contact opens; wired anywhere else there is no pull-down and the
fail-safe property in the table below does not exist.

### Why the sense comes from the 12 V and not from the button

The earlier design fed this circuit from the Pi's own 3.3 V through a **second**
NC contact on the mushroom. The mushroom has only one NC, and that one is
committed to the job that actually removes the hazard.

Tapping the 12 V costs no second contact and is a **truer signal**: it reports
that the laser feed is live, so it also goes LOW on a dead supply, a pulled 12 V
wire or a failed LRS-50-12 — conditions under which the system genuinely cannot
fire and must not report otherwise.

What it gives up is independence. The hardware interlock and the software sense
now derive from one contact, so a welded NC defeats both at once (§3, item 5),
and GPIO 25 cannot be exercised on the bench until the 12 V branch exists.

### Component notes

- **10 kΩ** series (NOT the 1 kΩ of the older revision): the source here is the
  12 V laser feed, not the Pi's 3.3 V rail, so the divider is the arm circuit's.
  With a 1 kΩ fitted the junction would sit at `12 × 3 / 4 ≈ 9 V`, leaving the
  Zener as the only thing between that and a 3.3 V input.
- **3 kΩ**: pull-down that forces LOW when the contact opens or a wire breaks.
  Taking `V_IH` at a conservative 2.0 V, the floor for this leg is ≈ 2 kΩ.
- **100 nF** + **BZX55C3V3**: debounce and transient clamp. **Cathode (banded
  end) at the junction, anode at GND.** Reversed it forward-clamps the junction
  near 0.7 V — below `V_IH` — so the pin reads "pressed" permanently and the
  system can never leave `SAFE_HALT`.
- **Meter both sense networks with the GPIO wire disconnected**, before either is
  attached to the Pi header. Arm OFF/ON must give ≈ 0 V / 2.77 V; E-stop
  pressed/released must give ≈ 0 V / 2.77 V. A wiring error found with a meter
  costs a minute; the same error found by the Pi can cost a GPIO pin.

### Operation

| State | E-stop contact | 12 V at the tap | GPIO 25 | Software interpretation |
|-------|----------------|-----------------|---------|-------------------------|
| Released (normal) | Closed | present | ≈ 2.77 V (HIGH) | System OK |
| Pressed | Open | absent | LOW (pulled down) | Emergency stop → `SAFE_HALT` |
| Sense wire broken | — | absent at the junction | LOW (pulled down) | Emergency stop → `SAFE_HALT` |
| Laser supply off or failed | — | absent | LOW (pulled down) | Emergency stop → `SAFE_HALT` |

Fail-safe: every one of these produces the same safe state.

**Consequence for start-up order.** GPIO 25 is HIGH only while the 12 V supply is
energised *and* the mushroom is released. The LRS-50-12 must therefore be plugged
in and running **before** the application starts, or the first `check()` reads an
E-stop — and `SAFE_HALT` is terminal. §15 sequences this, and it is the one
operational cost of sourcing the sense from the 12 V.

---

## 6a. Enclosure Door Interlock

A Class 4 enclosure must terminate access to the beam when it is opened. Earlier
revisions of this project referenced door-interlock *testing*
(`PRE_FLIGHT_CHECKLIST.md` §8) without ever specifying a circuit, which meant the
interlock existed only as an intention. This section is that circuit.

**The interlock now needs one NC contact, not two.** Under the contactor design
it needed two because there were two circuits to break — the K1 coil circuit and
the 12 V laser feed. With the contactor gone there is one circuit, so one contact
does the whole job:

```
Door NC ──► in series with the E-stop and the arm lever on the 12 V laser feed (§3)
            door open  →  laser driver +VIN removed
                       →  GPIO 24 reads disarmed (§5)
```

A second NC contact, if the switch has one, is **spare**. Do not invent a use for
it: a redundant copy of the same contact in the same conductor adds nothing,
because the conductor is already broken by the first one.

### This is a hardware-only interlock, and that is the stronger choice

There is no door GPIO and no code change. An interlock's job is to **remove the
hazard**, not to inform software, and a contact that opens the 12 V feed is
strictly stronger than a pin the control thread polls — the same reasoning
`AGENTS.md` §4.8 already applies to the arm switch, which is called out there as
"a true hardware interlock." Adding a software-visible door sense would mean a new
GPIO, a new HAL interface, a control-loop poll, state-machine wiring and tests, in
exchange for a signal that cannot do anything the contact has not already done.

It is nonetheless visible to the software for free: the door contact sits upstream
of the GPIO 24 sense tap, so opening the door reads as a disarm and the existing
`FiringController::set_armed(false)` path clears targets and rejects fire (§5).

### Switch requirements

- **At least one NC contact** with **positive/direct opening action**
  (IEC 60947-5-1 Annex K), so that a welded contact is still forced open by the
  door movement rather than relying on a spring.
- Rated for the laser driver's current at **12 V DC** — a DC rating, not an AC
  one (§3, "The mushroom's contact duty").
- Mounted so the actuator cannot be operated by hand while the door is open
  without a tool — an interlock that can be held closed with a fingertip is not
  an interlock.
- Actuated by the door itself, on the hinge-remote side, so a warped or partly
  closed door does not read as closed.

### Operational consequence

Opening the door removes laser-driver power and reads as a disarm. Closing it
restores power to the driver, but the software has already cleared its targets
and requires the arm path to come back before it can fire — and once the
`ArmSwitch` edge requirement of §3 is implemented, the lever must be cycled.
Any work needing the enclosure open is work that must not have a live laser
driver. Alignment is done with a verified low-power source inside the **closed**
enclosure (`PRE_FLIGHT_CHECKLIST.md` §5), which is why the interlock does not
obstruct the procedure it protects.

---

## 7. Raspberry Pi 5 GPIO Connections

| Function | GPIO | Pin | Direction | Voltage | Destination |
|----------|------|-----|-----------|---------|-------------|
| Laser TTL | GPIO 18 | 12 | Output | 3.3 V → SN74AHCT125N **#2** ch1 (§9) | 74HC123/74HC08 backstop, then verified laser TTL input |
| Arm switch sense | GPIO 24 | 18 | Input | 2.77 V HIGH | Arm switch voltage divider |
| E-Stop sense | GPIO 25 | 22 | Input | 2.77 V HIGH | 12 V tap after the E-stop NC + divider (§6) |
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
> board.
>
> **The marking is not sufficient evidence, and on the devices currently on hand
> it is not evidence at all.** They were bought under a marketplace brand from a
> trading company, with no manufacturer traceability (`HARDWARE_INVENTORY.md`),
> so the letters `AHCT` on the package are a claim by someone who did not make
> the die. An `AHC`/`HC` die in an `AHCT`-marked package is the cheap
> substitution and the pinout hides it completely. Read the marking **and** run
> the threshold test in §9.4.

#### Why this part cannot be qualified on the bench alone

This is the one device in the design whose value is a **datasheet guarantee**
rather than an observable behaviour, and it is worth being explicit about the
difference.

The 74HC123's die is checked by measuring its period (§11a): a wrong part shows
up in the number. The SN74HC08N is an AND gate whose function the §11a tests
exercise directly. But nothing you can do on a bench establishes
`V_IH = 2.0 V max at V_CC = 5 V across the full temperature range` — and that
specification, not any single observation, is what makes it legitimate to drive
this input from 3.3 V.

A threshold measured on one unit at room temperature is exactly the class of
evidence this section already rejects for direct wiring: *not valid merely
because it may appear to work on one bench unit*. An untraceable `AHCT` that
passes a bench threshold test sits in the same logical position as the direct
connection it was bought to replace.

**Obtain both packages from an authorised distributor** (Mouser, DigiKey, RS,
Farnell). The part is well under €1; the laser firing path runs through it. Use
the threshold test below on whatever is fitted, traceable or not — it still
catches an outright `HC` die.

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

- [ ] Every fitted device is marked `AHCT`, not `AHC` or `HC`. **Necessary, not
      sufficient** — see §9.1 on the traceability of the devices on hand.
- [ ] **Input threshold measured on every fitted device.** This is the test the
      marking cannot substitute for, and it is decisive against a remarked
      `AHC`/`HC` die. With `V_CC` = 5 V and `/OE` LOW, drive one input from an
      adjustable divider (a 10 kΩ potentiometer across the 5 V rail is enough)
      and raise it slowly while watching that channel's output:

      | Output flips at | Verdict |
      |-----------------|---------|
      | ~1.4–1.5 V | TTL thresholds — a genuine `AHCT` |
      | ~2.5 V (half `V_CC`) | CMOS thresholds — an `AHC`/`HC` die. **No-go** |

      Test both packages: they need not be from the same lot. Do **not** accept
      "it switched when I applied 3.3 V" as a pass — an `HC` part typically
      switches near 2.5 V and would pass that test while failing its own
      guaranteed 3.5 V `V_IH`. That is the failure this test exists to catch.
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

### What the board itself shows, and the one test that settles question 1

The vendor's annotated photo of this driver marks **three distinct connector
groups**. That much is now documented rather than assumed:

| Connector | Vendor label | Position on the board |
|-----------|--------------|-----------------------|
| Head, X axis | `HT-X21852` | top left; thick white motor/detector cable |
| Head, Y axis | `HT-Y21852` | top right; thick white motor/detector cable |
| Power | **±15V Power Supply** | centre, between the two bulk electrolytics |
| Signal, X axis | **±5V singal input** *(sic)* | left board edge |
| Signal, Y axis | **±5V singal input** *(sic)* | right board edge |

The board labelling its own power inlet **±15 V** is independent confirmation
that the kit supply is bipolar — the marking on the supply itself never said so.
It settles nothing about the signal topology: a vendor annotation is a claim, and
"±5 V input" is exactly as ambiguous on the board as it was in the listing.

**Question 1 can be settled today, unpowered, with a continuity meter.** Ring
every pin of one axis' signal connector against every pin of the ±15 V power
connector:

| Result | Meaning |
|--------|---------|
| **2 signal pins, neither continuous with a power pin** | genuine differential pair — §14 holds |
| **3 signal pins, exactly one continuous with the power `COM`** | differential pair **plus** a ground pin. Best case: that pin is where §13's common reference lands |
| **2 signal pins, one continuous with the power `COM`** | **single-ended input.** §14 does not hold. The complementary DAC pair would short one channel to ground, and a unipolar 0–5 V DAC cannot reach the negative half of the range in any case. **Stop and re-plan the analog stage** |

Do this before a DAC output is ever connected. It is the only one of the three
questions that can be answered without the manual, and it costs a minute.

**Result, 2026-08-09: three pins per axis, and the centre one is continuous with
the centre pin of the ±15 V inlet.** That is the third row — the best case. Two
consequences:

- **The single-ended front end is ruled out**, so §14 and the complementary DAC
  pair survive.
- **§13 gets simpler.** The driver's signal ground and the supply's `COM` are the
  same node, and that node is the **centre pin of the power connector** — now
  identified with no power applied. Bonding `COM` to the Pi's logic ground
  references the signal inputs at the same time: one wire, not two.

**Both outer pins then read finite to the centre (2026-08-09), so the
`IN`/`GND`/`NC` case is out as well.** Question 1 is now answered as far as a
meter can answer it: three pins, centre grounded, two live inputs — an
`IN+`/`GND`/`IN−` difference-amplifier front end.

Questions 2 and 3 still need the manual or the stage-8 check of §17, but they are
no longer **connection** risks. Under either reading of "±5 V" the complementary
DAC pair sits inside the envelope: each channel stays within 0–5 V to ground, and
the difference stays within ±5 V. What remains uncertain is the **scale** — how
many degrees the driver gives per volt of difference. A scale error is exactly
what `CALIBRATION.md` measures, by commanding a known angle and reading the
actual deflection. Connecting the DACs cannot damage the input; it can only
produce the wrong number of degrees, and that is caught long before any laser is
fitted.

### Three practical consequences for the wiring

**The board has a single ±15 V inlet, and the kit's DC harness has two legs.**
That harness is sized for the two-single-board variant of this kit; with this
dual board **one leg is left over and must be insulated.** A live ±15 V housing
loose against the chassis is a short straight across the supply.

**The `±5 V signal input` connectors are fed by the MCP4922 outputs, not by the
Pi.** The Pi has no analog output at all; the chain is Pi → SPI → `AHCT125` #1 →
the two MCP4922s → these two connectors. Nothing from a GPIO header ever lands
here — a 3.3 V logic pin on an input built for ±5 V analog is a wiring error, not
a shortcut. The centre pin of each connector is the ground already shown to be
common with the supply's `COM`, so §13's single bond references both axes.

**Which outer pin is `IN+` and which is `IN−` is not established.** Getting it
backwards inverts that axis: the mirror moves the wrong way. It damages nothing
and is fixed by swapping the two wires. Resolve it at `CALIBRATION.md` by
commanding a known deflection — not by guessing from wire colour. The same
applies to which edge connector is X and which is Y: the vendor photo puts X
beside `HT-X21852`, but that is worth one command to confirm.

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
12 V + (LRS-50-12) ─┬─► [E-STOP NC, §3] ─► [ARM switch] ─► [DOOR NC, §6a] ─► Laser driver +VIN
                    │
                    └─► Laser cooling fan   ← upstream of all three contacts, see below
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

- The laser driver receives **no power** until all three series contacts of §3 are closed: the E-stop released, the arm switch ON, and the enclosure door closed (§6a). These are the hardware interlocks, and any one of them opening removes `+VIN`. **None of them latches** — see §3 for what that costs.
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
    74HC123  1Cext (pin 14) ──── 1 µF / 50 V monolithic ceramic ──── 1Rext/Cext (pin 15)
    74HC123  1Rext/Cext (pin 15) ── 220 kΩ ──► +5 V
    74HC123 VCC (pin 16) ──► +5 V ; GND (pin 8) ──► common GND
    100 nF ceramic directly across 74HC123 pins 16↔8
    Unused channel 2: hold 2CLR/2RD (pin 11) LOW; tie 2A/2B (pins 9/10) to defined levels;
                      leave 2Cext (pin 6) and 2Rext/Cext (pin 7) open

AND gate:
    SN74HC08N VCC (pin 14) ──► +5 V ; GND (pin 7) ──► common GND
    100 nF ceramic directly across SN74HC08N pins 14↔7
    Tie every unused AND-gate input to a defined HIGH or LOW; leave unused outputs open
```

> **The 220 kΩ goes to pin 15.** On the standard 74HC123 pinout the two channels
> follow the *same* order, not a mirrored one: `6 = 2Cext, 7 = 2Rext/Cext` for
> channel 2, and `14 = 1Cext, 15 = 1Rext/Cext` for channel 1 — Cext at the lower
> pin number in both. This is confirmed against the pin arrangement in the
> Hitachi **HD74HC123A** datasheet, the family of the device actually on hand.
>
> Two earlier revisions of this document had 14 and 15 the other way round, the
> second one justified by a "mirrored channels" claim that the datasheet does not
> support. Built that way the 220 kΩ lands on the capacitor-only node instead of
> the timing node: the one-shot does not time as designed, and the mandatory
> measurement below then corresponds to nothing in the design. Check it against
> the datasheet of the device actually in your hand before powering the board.

Pin 13 is the channel-1 **active-HIGH Q** output on the standard 74HC123 pinout;
pin 4 is its active-LOW complement and must not be used for the AND gate. Verify
the full marking on the on-hand IC against its manufacturer's datasheet before
wiring. Holding 1A LOW is required for a rising edge on 1B to trigger the
monostable. Tying 1CLR/1RD to the translated fire level holds the channel reset
while idle and allows the rising fire transition to arm/trigger it; this exact
behaviour must be included in the tests below.

The `1Y → 74HC08` output pull-down (c) is what holds the **laser driver's own**
TTL pin LOW when the 74HC08 is unpowered, removed from its socket, or its output
cable is broken. Pull-downs (a) and (b) protect the translator; neither reaches
past the AND gate. Size (c) once the laser driver's TTL input has actually been
characterised (§11): 10 kΩ is the starting value, but if that input carries an
internal pull-up, (c) must be low enough to hold it below the driver's `V_IL`
with the 74HC08 absent, while still letting the 74HC08 pull it HIGH.

### Timing

The on-hand device is a **Hitachi HD74HC123P** (date code 9K06). Its datasheet
states the pulse equation with **no coefficient at all**:

`t_W = R_ext · C_ext`

and the switching table confirms it — `C_ext = 0.1 µF, R_ext = 10 kΩ` is
specified as a typical `t_WQ` of **1.0 ms**, which is exactly `R · C`. So for the
fitted parts:

`t_W = 220 kΩ · 1 µF ≈ 220 ms`

**This is not the 99 ms this document previously claimed.** That figure came
from applying the Texas Instruments `K = 0.45` coefficient to a part that is not
a TI part — an assumption recorded as an assumption, and wrong by a factor of
2.2 in the direction that *weakens* the backstop. At 220 ms the one-shot sits
**above** the ~105 ms real software bound rather than just below it, so it no
longer clips legitimate pulses and instead trips only on a genuine control-thread
failure — but with far more slack than intended, holding 2.5 W on for over twice
the software limit before cutting.

Choose the regime deliberately and size the network for it, all values with
`K = 1`:

| Intent | Target | With C = 1 µF | With R = 220 kΩ |
|--------|--------|---------------|-----------------|
| Hardware is the binding limit, clips long pulses | ~99 ms | R = 100 kΩ | C = 0.45 µF |
| Trips only on failure, modest slack | ~130–150 ms | R = 130–150 kΩ | C = 0.6–0.68 µF |
| As currently fitted | ~220 ms | — | — |

**Measure the assembled circuit** whatever you choose. The datasheet gives the
equation, not the part in your hand: the capacitor's dielectric is unrecorded,
and a Y5V or Z5U 1 µF can deliver a fraction of its nominal capacitance under
bias and temperature. Any change to the timing network requires a new
calculation and a fresh measurement.

The marking reads `HD74HC123P`, without the `A` of the datasheet revision
obtained (`HD74HC123A`). The pin arrangement is the industry-standard 74123
pinout and is not in doubt; the coefficient should be re-confirmed by the
measurement, which is mandatory regardless.

**±20% is accurate enough — this is a bound to be known, not a parameter to be
tuned.** What the measurement rules out is not a 10% error but a
factor-of-three one. The vendor coefficient is unrecorded, and "1 µF 50 V
monolithic ceramic" does not state a dielectric: a Y5V or Z5U part can deliver a
fraction of its nominal capacitance under bias and temperature, which puts t_W
nearer 30 ms than 99 ms. A short period is not a safety failure — the backstop
only gets tighter — but it silently truncates every legitimate pulse, so the
system delivers a fraction of the energy the operator believes it does. A long
one makes the backstop weaker than the documentation claims. Both are invisible
without a number.

### Measuring the period

This belongs to **stage 4** of §17: translator, one-shot and AND gate on the
bench, powered from the Pi's 5 V header rail, with **no laser of any class
connected** — the laser-driver TTL connector goes nowhere at this stage. No
mains, no ±15 V, no 12 V.

**Nothing is rearranged for the measurement.** The timing network, the three
fail-LOW pull-downs, the trigger pins and the decoupling all stay in their final
configuration, because a period measured on a different circuit is a period that
does not apply to this one. A breadboard is acceptable: its stray capacitance is
picofarads against a 1 µF timing capacitor — one part in a hundred thousand,
well under the tolerance of the capacitor itself.

**With a scope or logic analyser.** One channel on the 74HC08 1Y output (pin 3),
×10 probe, trigger on the rising edge, timebase ~20 ms/div. Nothing else needed;
the probe tolerates the 5 V node directly.

**With the Raspberry Pi itself.** A ~100 ms interval does not need a scope. Add
a temporary 2:1 divider from the AND output to a spare GPIO — the output is 5 V
logic and would damage a 3.3 V pin if connected directly:

```
74HC08 1Y (pin 3) ──┬── 10 kΩ ──────► GND          pull-down (c), already fitted
                    │
                    ├── 10 kΩ ──┬─── 10 kΩ ──► GND  temporary measurement divider
                    │           │
                    │           └──► GPIO 23 (header pin 16), reads 2.5 V
                    │
                    └── laser-driver TTL connector — NOT connected at stage 4
```

The divider sits in parallel with pull-down (c), presenting 6.67 kΩ to the
74HC08 — 750 µA at 5 V, far inside its drive capability — and it touches nothing
upstream, so the timing network is unaffected.

Monitor **both** edges on the sense pin. The AND output rises when GPIO 18 is
asserted and falls when Q times out, so the interval between the two timestamps
is `t_W` directly, with nothing to correlate against.

```bash
gpiodetect                  # the header chip is the one with ~54 lines; on a
                            # Pi 5 it is not always gpiochip0
gpiodetect --version        # 1.6.x uses the syntax below; 2.x renames the tools

# terminal 1 — watch both edges of the laser TTL
gpiomon --format="%e  %s.%n" gpiochip0 23

# terminal 2 — GPIO 18 HIGH and held, simulating a hung control thread
gpioset --mode=wait gpiochip0 18=1
```

Terminal 1 prints a rising event and, one `t_W` later, a falling event; the
difference is the period. Scheduling jitter on a non-RT kernel is of the order
of a millisecond — **1% of 100 ms**, an order of magnitude better than this
measurement needs.

`gpioset` drives GPIO 18 through the same libgpiod character-device interface
the application uses, so this exercises the real path rather than a simulation
of it. It also claims the line exclusively: **the application must not be
running.**

The same bench answers the two pass/fail conditions below, which matter more
than the number: release GPIO 18 immediately instead of holding it, and the
output must be equally brief (the AND gating is present); hold it, and the
output must produce exactly one pulse and then stay LOW (the backstop cuts).

**Remove the measurement divider afterwards.** Left fitted it would hand
software a reading of the actual laser TTL state — precisely the kind of signal
`AGENTS.md` §4.5b argues against, since it informs the control thread of
something it cannot act on that the hardware has not already done. Pull-downs
(a), (b) and (c) stay: those are the circuit.

### Verification (mandatory before connecting the Class 4 laser)

- [ ] **Short pulse passes through:** drive GPIO 18 HIGH for ~10 ms; the laser TTL must be HIGH for ~10 ms (not stretched to the full one-shot width, ~220 ms as fitted). If it stretches, Q is driving the TTL *alone* — the AND gate is missing or miswired. Fix before proceeding.
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
- [ ] **Timing pins verified against the datasheet:** 220 kΩ on **pin 15**
  (1Rext/Cext), capacitor between pins 14 and 15. Wired to pin 14 instead, the
  resistor feeds the capacitor-only node and the measured period is meaningless.
- [ ] **Period measured** and recorded; matches intent (§ Timing above).
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

**Nothing sequences hazardous power for you.** Under the abandoned contactor
design the supplies stayed off until a deliberate START press. With no
contactor, plugging in energises both supplies, and the laser driver becomes
live the moment the three series contacts of §3 are all closed. The order below
is therefore an **operator procedure**, and its first step is the one that
matters.

### Power-up

1. **Arm lever OFF. E-stop pressed. Door closed.** Do this *before* anything is
   plugged in — these contacts are the only thing between a plugged-in supply
   and a live laser driver.
2. Plug in the **LRS-50-12** and the **±15 V supply**. Both are now live and stay
   live until unplugged. The galvo driver has power; the laser driver does not,
   because the E-stop and the lever are open.
3. The Pi boots from its own USB-C supply. The 5 V logic rail comes up with it,
   so the §9.3 pull-downs and the AND gate are defined from the first instant.
4. Confirm with a meter, before the software runs:
   - laser-driver TTL connector: **LOW**
   - all four DAC outputs: **≈ 2.5 V** (against the measured rail, §8)
   - GPIO 24 ≈ 0 V and GPIO 25 ≈ 0 V — the E-stop is still pressed
5. **Release the E-stop.** GPIO 25 must rise to **≈ 2.77 V**. GPIO 24 stays at
   0 V because the lever is still OFF, and the laser driver is still unpowered.
6. Start the application. Verify both camera streams open and that
   `[CONFIG] Aborting` does not appear. GPIO 25 must already be HIGH by now:
   `SAFE_HALT` is terminal, so an application started against a pressed E-stop or
   an unplugged 12 V supply dies immediately and permanently (§6).
7. Put on safety eyewear. Confirm the enclosure is closed and the beam path is
   contained.
8. Turn the arm lever **ON** only when ready to run. **This is the moment the
   laser driver first receives power.**

Steps 4 and 5 precede step 6 by design: the logic demonstrates a centred galvo
pair and a LOW laser TTL *before* the software can command anything.

### Power-down

1. Turn the arm lever **OFF**. The laser driver loses power.
2. Wait 10 seconds — the firing cooldown, and time for the fan to pull heat out
   of the module (the fan runs while the 12 V branch is live, §11).
3. Trigger software shutdown (`sudo shutdown now`).
4. After the Pi halts, **unplug both supplies.**

### Emergency

Press the mushroom at any time. The laser driver loses `+VIN` and cannot emit.
GPIO 25 goes LOW and the software enters `SAFE_HALT`. The Pi keeps running on its
own supply, so the log of what happened survives.

**Releasing the mushroom restores the laser driver's 12 V. There is no latch in
this build** (§3). Nothing fires on release, because the software is in the
terminal `SAFE_HALT` state with GPIO 18 LOW and the §9.3 pull-downs holding the
TTL line down — *unless* the control thread had hung with GPIO 18 HIGH, in which
case the laser emits for one 74HC123 one-shot period before the backstop cuts
it.

**So after any E-stop: turn the arm lever OFF before releasing the mushroom.**
This is a procedure, not a hardware guard, and it is the weakest link in this
architecture. It exists because nothing enforces a deliberate restart until the
`ArmSwitch` LOW→HIGH edge requirement of §3 is implemented.

`SAFE_HALT` is terminal, so recovery also requires restarting the software.
Restarting it with the mushroom released and the arm lever still ON takes the
state machine from `INIT` to `FIRING` with no deliberate action anywhere in the
sequence.

### For any work inside the enclosure

**Unplug both supplies.** The mushroom is not an isolator — mains and both DC
supplies stay live while it is pressed. An E-stop is not a lock-out.

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
following it as one would put a live laser feed on the bench before the
interlock chain and the pulse-duration backstop have been proven.

Build in **risk order** instead. Each stage has a gate; do not start the next
stage until the gate passes.

**Stages 0 to 5 require no mains and no 12 V at all.** The 5 V logic rail comes
from the Pi header (§4), so the whole logic chain and both sense networks can be
built and metered on SELV alone.

| Stage | Work | Gate |
|-------|------|------|
| **0** | **Freeze and inspect.** Reconcile every drawing against this document. Meter every resistor against its documented value (§2). Read the marking on every IC (`AHCT`, not `AHC`/`HC`) and threshold-test both `AHCT125`s per §9.4, since their marking is untraceable. Ring out the mushroom, lever and door contacts, and check the mushroom NC for a **DC rating** and the **Annex K** direct-opening symbol (§3). Obtain the galvo driver manual (§10). | Every part identified by measurement rather than by the bag it came in; §10's three questions answered from the manual |
| **1** | **Enclosure mechanics.** Case, beam dump, door-interlock switch mounting, PE bonding plan. No electronics. | Door interlock actuates on door movement and cannot be defeated without a tool (§6a) |
| **2** | **Pi alone.** USB-C only. Toggle GPIO 18/24/25, watch edges with `gpiomon`. No 5 V rail, nothing else connected. | Every pin behaves; `gpiomon` timestamps are sane |
| **3** | **DAC island.** 5 V logic rail, AHCT125 #1, both MCP4922s **including `/LDAC` to GND and `/SHDN` to +5 V**, CE0/CE1 pull-ups. | All four DAC outputs ≈ 2.5 V; the 5 V rail measured at the DAC pin under load and written into `dac_reference_voltage` (§8) |
| **4** | **Pulse-duration backstop.** AHCT125 #2, 74HC123 with the **220 kΩ on pin 15**, SN74HC08N, pull-downs (a)(b)(c). Instrumentation and dummy load only — no laser of any class. A scope, a logic analyser or the Pi's own `gpiomon` all work; the method is in §11a. | Every checkbox in §11a passes and **the measured period is recorded.** In this build that number bounds the beam an E-stop *release* can produce after a hung control thread (§3), so it is a safety figure, not a design check |
| **5** | **Sense networks, built and metered.** Both dividers are now identical — 10 kΩ series, 3 kΩ lower leg, 100 nF, Zener band at the junction. Feed each tap from a bench source with the GPIO wire **disconnected**. | Each junction ≈ 0 V with its tap open and ≈ 2.77 V with 12 V on the tap (§5, §6) |
| **6** | **Mains tails and DC supplies, unloaded.** Land `L`/`N`/`⏚` on both supplies from factory-moulded cords and fit the LRS-50-12's terminal cover. The galvo supply is an open-frame board with pin headers and **no cover** (§2): identify its mains end by ringing a header pin to a leg of `F1` **unplugged**, wire it unplugged, land the meter on the *wire ends* away from the PCB, and only then energise. Nothing connected downstream. | 12 V within tolerance; +15 V, 0 V and −15 V within tolerance **and `+15`→`−15` about double `+15`→`COM`** — anything else means the supply is not bipolar and is a no-go for the driver; PE continuous from the plug to both supply chassis, the enclosure and both driver chassis (§13) |
| **7** | **Interlock chain, with a dummy load.** E-stop NC, arm lever and door contact in series in the `12 V+` conductor, a resistive dummy load in place of the laser driver, both sense taps connected to the Pi. | Opening **any** of the three kills the dummy load. GPIO 25 goes LOW on the E-stop **only**; GPIO 24 goes LOW on any of the three. Releasing the mushroom restores the dummy load — confirm that is understood and that §15's lever-first procedure is written on the enclosure (§3) |
| **8** | **Galvo driver, no head.** Supplies, ground, and the analog inputs per the topology verified at stage 0. | Commanded DAC pairs produce the expected differential at `IN+`/`IN−` with no clipping or offset (§10, §14) |
| **9** | **Galvo head connected, no optical source.** | Mirrors centre on start-up and slew to commanded angles without hitting a mechanical limit |
| **10** | **Cameras and calibration.** By-path identification and labelling, stereo calibration, dry-run tracking. | `PRE_FLIGHT_CHECKLIST.md` §3 and §4 complete |
| **11** | **Verified low-power alignment source**, inside the **closed** interlocked enclosure. | `PRE_FLIGHT_CHECKLIST.md` §5 complete |
| **12** | **Class 4 module.** | Only after a final electrical, laser-safety and functional-safety review, with eyewear, beam dump and every no-go item in `HARDWARE_INVENTORY.md` closed — including the `ArmSwitch` edge requirement of §3, which is the only thing that makes a restart deliberate |
