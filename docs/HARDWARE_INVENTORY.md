# Reported Hardware Inventory

Every part the design calls for is now reported as physically on hand
(**procurement closed 2026-08-03**; the earlier list closed 2026-07-29).

**Possession is not verification, and this file is not a go-ahead.** Nothing
below has been metered, ringed out, marking-checked, timed, or read
against its manufacturer's own documentation, and the mains package has not
been built. Every one of those gates is still open and is tracked in the
reconciliation table below — a complete parts bin changes what is *missing*,
not what is *proven*. Vendor-listing values remain vendor-listing values.

Detailed operating parameters and derived limits remain in
[`HARDWARE_PARAMETERS.md`](HARDWARE_PARAMETERS.md); the intended circuit is in
[`HARDWARE_WIRING.md`](HARDWARE_WIRING.md).

## Optics, motion, and power

| Item on hand | Qty | Reported specification |
|--------------|-----|------------------------|
| X-Y galvo kit | 1 | “20Kpps Laser Galvo X-Y Scanning Galvanometer SLA 3D DIY Animation Stage Light”; 400–700 nm mirrors; head rated ±12 V; driver requires ±15 VDC and accepts ±5 V analog input at 0.33 V/° |
| Stereo cameras | 2 | OV9281 global-shutter monochrome camera modules; module/firmware ID, USB mode list, fitted lens focal length, and lens distortion have not yet been recorded |
| Working laser module | 1 | 2.5 W, 450 nm blue, 33 × 70 mm; 12 VDC external “ACC constant-current” drive (seller wording); forced-air cooling; 3-pin TTL switching/PWM power-control interface; anodized-aluminium shell; coated optical-glass collimator |
| Working-laser PSU | 1 | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W |
| Galvo-driver PSU | 1 | Bipolar ±15 VDC supply; make/model, rated current per rail, and the three-conductor output (+15 V, 0 V/COM, −15 V) not yet recorded |
| Alignment/test laser | 1 | 5 mW, 12 mm module; wavelength, supply voltage, pinout, modulation interface, and labelled laser class not yet recorded |
| Enclosure | 1 | Laser-safe interlocked case with beam dump; rating against 450 nm at 2.5 W not yet recorded |
| Safety eyewear | 1 per person | Rated for 450 nm; OD figure and marking not yet recorded |

The product listing also advertised higher-power laser variants. They are not
part of this inventory; all project documentation and safety controls refer to
the reported **2.5 W** module only.

## Control and interlock parts

| Item on hand | Qty | Reported specification |
|--------------|-----|------------------------|
| DAC | 2 | MCP4922, DIP-14, dual-channel 12-bit DAC |
| Level translation | 2 | SN74AHCT125N, PDIP-14 quad bus buffer with 3-state outputs — package #1 for the four SPI signals, package #2 for GPIO 18 alone. **Marking not yet checked on either device** |
| Monostable | 1 | **Hitachi HD74HC123P**, DIP-16, date code 9K06 — identified from the package marking 2026-08-03. Marked without the `A` of the `HD74HC123A` datasheet revision obtained |
| AND gate | 1 | SN74HC08N, DIP-14, quad 2-input AND gate |
| Resistors | count not recorded | Carbon film, 1/2 W: 220 kΩ, 10 kΩ, 3.3 kΩ and 1 kΩ values |
| Capacitors | count not recorded | 50 V monolithic ceramic: 1 µF and 100 nF values, plus a 10 µF rail bulk capacitor |
| Zener diode | count not recorded | BZX55C3V3, DO-35 axial, 0.5 W, 3.3 V |
| Arm control | 1 | Lever switch; contact arrangement and DC current rating not recorded |
| Emergency control | 1 | Mushroom button; pole count, NC/NO contact arrangement, and mains rating not recorded |
| Door interlock | 1 | Switch intended to provide 2 independent NC contacts with positive/direct opening action; markings and actuator type not recorded |
| Safety contactor **K1** | 1 | 2 main poles plus auxiliary; pole rating against the combined cold-start inrush, coil voltage, and auxiliary contact count not recorded |
| START / STOP buttons | 1 each | START NO momentary, STOP NC momentary, for the K1 coil circuit (§3) |
| Mains isolator + protection | 1 set | Isolator, overcurrent protection and RCD; ratings and coordination not recorded |
| Mains terminal blocks | 1 strip | DIN-rail terminal blocks, one block per net (§3) |
| Wiring connector | count not recorded | WAGO 221-413 lever connector, 3-conductor, max 4 mm² |

`AHCT` is load-bearing and is **not** interchangeable with `AHC` or `HC`. At
V_CC = 5 V, `AHCT` has TTL input thresholds (V_IH = 2.0 V) and accepts a 3.3 V
Raspberry Pi output with margin; `AHC` and `HC` require 0.7 × V_CC = 3.5 V and
reproduce the exact problem the part exists to solve. All three families share
the same pinout, so a wrong-family substitution is invisible on the assembled
board. Order spares and check the marking on every fitted device.

Wiring, pin assignment, the three mandatory laser-path pull-downs, and the
verification steps are in `HARDWARE_WIRING.md` §9.

## Inventory-to-design reconciliation

Nothing here is a purchase any more. Every remaining gap is a **measurement, a
document to read, or work to be performed** — and each is still an explicit
**no-go item**. Do not substitute or bypass a safety circuit to make the
on-hand parts fit, and do not treat a full parts bin as a closed gate: the
whole reason this table survived procurement is that a part which is present
but unverified fails in exactly the ways a missing part cannot, because you
believe it is there and working.

| Gap | Why it matters | Required resolution before powered assembly |
|-----|----------------|---------------------------------------------|
| **The design needs 6 × 10 kΩ, not 3** | One arm series resistor, **three** laser-path fail-LOW pull-downs — one per node: translator input, 74HC123/74HC08 inputs, and the laser-driver connector (`HARDWARE_WIRING.md` §9.3) — and two chip-select pull-ups (§8). The third pull-down is the only thing holding the driver's own TTL pin LOW when the 74HC08 is unpowered, removed, or its cable is broken; (a) and (b) are both on the translator side and do not reach past the AND gate. | Count and meter-check **6 × 10 kΩ, 1/2 W** out of the bin. Fit each one physically adjacent to the input it protects — a pull-down at the far end of a broken wire protects nothing. |
| Level translation for the SPI and laser paths | Direct 3.3 V drive is not guaranteed into either a 5 V MCP4922 (V_IH = 3.5 V) or the 5 V 74HC123/74HC08 backstop (V_IH = 3.5 V). The translator must be **push-pull** with TTL input thresholds, fast enough for a 50 ns bit period at 20 MHz, and must cover **five** signals — four SPI plus GPIO 18. | **2 × SN74AHCT125N are on hand.** Open until both devices are marking-verified as `AHCT` — not `AHC`, not `HC`, which are pin-identical and would reproduce the exact fault the part exists to remove — fitted with **all three** fail-LOW pull-downs (`HARDWARE_WIRING.md` §9.3), scope-verified per §9.4 — the 20 MHz SPI edges genuinely need one — and bench-verified per §11a, where the ~100 ms one-shot does not. The pin-identical **SN74AHCT126N** with a self-gated active-HIGH `OE` was considered for the laser path and **rejected**: it adds no guarantee that pull-down (b) does not already provide, and it makes the *normal idle* laser TTL a resistive LOW instead of a driven one — reasoning recorded in §9.3. Fit only `AHCT125`. |
| **Mains switching architecture is not built** | `HARDWARE_WIRING.md` §3 no longer passes mains through the mushroom contact, for two independent reasons. **Inrush:** the LRS-50-12 alone specifies a 45 A cold-start at 230 VAC, and the ±15 V supply adds its own; a pilot-duty contact block subjected to that welds rather than fails open, leaving an E-stop that looks normal and disconnects nothing. **Automatic restart:** with mains through the NC contact, *releasing* the mushroom re-energises everything. Because `SAFE_HALT` is terminal, this project's own documented recovery is "restart the software" — done with the mushroom released and the arm switch still ON, that walks `INIT → ARMED → FIRING` with no deliberate start action. | K1, START, STOP, the isolator/OCP/RCD set and the DIN terminal blocks are on hand; **none of it is wired.** Have a qualified person build and inspect the assembly, and confirm against the parts actually purchased that K1's main poles are rated for the *combined* inrush, that its coil matches the control circuit, and that it carries ≥1 auxiliary NO for the seal-in path. Then decide the integrity level: latching contactor as the minimum, or a monitored safety relay with **mirror-contact** feedback (IEC 60947-4-1) that detects a welded main pole and refuses the reset. That choice is a functional-safety decision and is itself open. Commission by verifying that the E-stop drops K1, that **releasing it does nothing**, and that both DC rails measure 0 V while it is pressed. |
| **Double-pole isolation is required, and the inlet polarity cannot be assumed** | An earlier revision instructed "wire only the Live wire through the E-Stop". On a reversible inlet — CEI 23-50 (Italian Type L), Schuko/CEE 7 — the conductor labelled "L" is Live only half the time it is plugged in, so single-pole switching can break Neutral and leave both supplies live-referenced behind an E-stop that reads as open. | K1 must break **both** live conductors. PE is never switched and never fused. Confirm the intended inlet/plug type and have the isolation design reviewed by the qualified person doing the mains work. |
| **Enclosure door interlock is not installed or verified** | A Class 4 enclosure must terminate beam access when opened. The switch is on hand but nothing has confirmed it actually has **two independent NC contacts with positive/direct opening action** (IEC 60947-5-1 Annex K) and a tool-required actuator — a switch sold as an interlock and a switch that meets Annex K are not the same claim, and the difference is invisible until it fails to open. | Confirm the contact arrangement and the positive-opening marking from the switch's own documentation, then wire per `HARDWARE_WIRING.md` §6a: contact 1 in the K1 coil circuit, contact 2 in series with the arm switch on the laser 12 V feed. Prove it by opening the door and watching the disarm. Hardware-only by design — no GPIO and no code change. |
| Mushroom-button contacts are unspecified | Two independent NC contacts are still required: pole 1 in the K1 coil circuit, pole 2 for the GPIO 25 sense circuit. Pole 1's duty is now only the coil current, which is far easier to satisfy than the original mains duty — but it must still be rated for it. A monitored-safety-relay architecture would need dual-channel E-stop inputs and therefore a **third** contact, or the GPIO 25 sense would move to a spare K1 auxiliary. | Ring out the contacts and verify markings/datasheet for 2×NC operation and the coil-circuit rating. If the button on hand turns out not to have two independent NC poles, it is replaced — not worked around. If the safety-relay option is chosen, confirm the contact count first. |
| Lever-switch contacts are unspecified | The switch is intended to interrupt 12 V laser-driver power, not merely provide a logic input. It is now in series with the door interlock's second contact. | Ring out the contacts and verify the DC voltage/current rating against the *measured* laser-module current. DC breaking capacity is the figure that matters here and it is often absent from a switch sold on its AC rating. |
| ±15 VDC galvo supply is unmeasured and unbonded | The driver needs a genuine bipolar supply whose output is **three conductors** — +15 V, 0 V/COM, −15 V — where COM is the reference the differential inputs measure against. A supply on hand is not a supply confirmed to deliver both rails within tolerance, nor one whose COM has been bonded. The 12 V Mean Well remains the laser branch and is not a substitute. | Measure all three conductors unloaded (`HARDWARE_WIRING.md` §17 stage 7), confirm the current rating covers both driver channels, and bond COM into the common signal reference (§13). |
| **Galvo-driver analog input topology and common-mode range are unverified** | This is the assumption with the widest blast radius in the design: `HARDWARE_WIRING.md` §14, `CoordinateMapper`, `DifferentialGalvoDriver` and the choice of two MCP4922s all rest on it. Because the DAC channels are complementary unipolar 0–5 V outputs, the pair presents a permanent **2.5 V common mode**. "±5 V analog input at 0.33 V/°" in a seller listing establishes neither that the inputs are a genuine differential pair, nor that 2.5 V is inside the permitted common-mode range, nor that ±5 V is a *differential* rating rather than per-input-to-ground. If any is wrong, every commanded angle is wrong by a scale or offset error — a silent aiming fault. | Obtain the driver's own manual or schematic and confirm all three points before connecting the DACs. Re-confirm empirically during calibration: command a known angle and measure the actual deflection. Do not resolve this by trial with any laser connected. |
| Test-laser electrical details are unknown | A bare 12 mm, 5 mW module may not accept the working laser’s 3-pin TTL interface and therefore may not test the real gating chain. | Record its wavelength, labelled class, supply voltage, current, pinout, and TTL behaviour. Use it for gated alignment only if electrically compatible; otherwise use a suitably classified TTL-controlled alignment module. |
| Working-laser 3-pin pinout and TTL levels are unknown | “TTL/PWM” in a seller listing does not establish pin order, active polarity, input thresholds, or safe power-up state. **Polarity is the critical unknown:** every fail-LOW measure in `HARDWARE_WIRING.md` §9.3 and §11a assumes LOW = off. On an active-LOW input the whole chain inverts and all three pull-downs become a *fire* command — the pull-downs would have to become pull-ups and the AND gate would have to become a different gate. | Obtain the module/driver pinout and confirm the input is **active HIGH** from its own documentation. Verify with the Class 4 optical output disconnected or blocked by an appropriate non-optical test load. Do not establish polarity experimentally with the Class 4 module connected. The project firing path must remain single-level TTL, never PWM. |
| Passive-component quantities/tolerances are still unmeasured | The design needs a timing R/C, per-IC supply decoupling, rail bulk decoupling, two debounce capacitors, two Zener clamps, three laser-path fail-LOW pull-downs, and two chip-select pull-ups. The actual 74HC123 pulse width depends on the fitted R/C values; the vendor is now known (Hitachi, `t_W = R·C`), which puts the nominal at ≈220 ms. | Count and meter-check parts against the running totals in `HARDWARE_WIRING.md` §2: **6 × 10 kΩ**, **8 × 100 nF**, 2 × 3.3 kΩ, 1 × 1 kΩ, 2 × BZX55C3V3, 1 × 10 µF, 1 × 220 kΩ, 1 × 1 µF. Fit 100 nF at each IC, and measure the one-shot period before connecting either laser. |

WAGO 221-413 connectors are three-conductor splicing connectors, not barriers
between unrelated circuits. Use a separate connector for each of Live, Neutral,
PE, and each DC net; install them inside an appropriate enclosure with the
required strain relief and segregation.

## Manufacturer references used for reconciliation

These references support the interface checks above; they do not identify the
manufacturer of an unmarked on-hand part.

- [Microchip MCP4902/4912/4922 data sheet](https://ww1.microchip.com/downloads/en/devicedoc/22250a.pdf) — `/LDAC`, `/SHDN`, and the BUF = 0 reference range
- [TI SN74AHCT125 product information](https://www.ti.com/product/SN74AHCT125)
- [TI SN74AHCT126 product information](https://www.ti.com/product/SN74AHCT126) — the pin-identical active-HIGH `OE` variant, considered and rejected
- **Hitachi HD74HC123A data sheet** — the family of the device actually on hand.
  Two things come from it and neither matches what this project previously
  assumed from a TI part: the pulse equation is `t_W = R_ext · C_ext` with **no
  coefficient** (confirmed by its own switching table, where 10 kΩ × 0.1 µF is
  specified as a typical 1.0 ms), and the channel-1 timing pins are
  **14 = 1Cext, 15 = 1Rext/Cext** — the *same* order as channel 2, not mirrored
- [TI SN74HC08 product information](https://www.ti.com/product/SN74HC08)
- [Mean Well LRS-50 series data sheet](https://www.meanwell.com/Upload/PDF/LRS-50/LRS-50-SPEC.PDF) — the 45 A cold-start inrush figure that rules out direct mains switching through a pilot-duty contact
- [WAGO 221-413 product information](https://www.wago.com/global/installation-terminal-blocks-and-connectors/splicing-connector-with-levers/p/221-413) — a three-port *splicing* connector, not a terminal block and not a barrier between circuits

Standards referenced for the E-stop and interlock functions. None is a legal
obligation for a one-off self-built rig that is not machinery placed on the
market; they are cited as the design standard, and the engineering reasons are
stated on their own terms in `HARDWARE_WIRING.md` §3 and §6a.

- **ISO 13850** — emergency stop function; the directly applicable reference.
- **Machinery Directive 2006/42/EC**, Annex I §1.2.4.3 — disengaging the E-stop
  must not restart the machinery, only permit restarting. In force until
  20 January 2027, when Regulation (EU) 2023/1230 replaces it.
- **IEC 60947-5-1** Annex K — positive/direct opening action for interlock
  contacts.
- **IEC 60947-4-1** — mirror contacts, for detecting a welded contactor pole.
- **IEC 60825-1** — laser product safety classification and enclosure
  requirements.
