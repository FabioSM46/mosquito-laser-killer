# Reported Hardware Inventory

Every part the design calls for is now reported as physically on hand
(**procurement closed 2026-08-03**; the earlier list closed 2026-07-29).

**Possession is not verification, and this file is not a go-ahead.** Nothing
below has been metered, ringed out, marking-checked, timed, or read
against its manufacturer's own documentation, and no mains conductor has been
landed. Every one of those gates is still open and is tracked in the
reconciliation table below — a complete parts bin changes what is *missing*,
not what is *proven*. Vendor-listing values remain vendor-listing values.

Detailed operating parameters and derived limits remain in
[`HARDWARE_PARAMETERS.md`](HARDWARE_PARAMETERS.md); the intended circuit is in
[`HARDWARE_WIRING.md`](HARDWARE_WIRING.md). **Readings actually taken on this
build are logged in [`MEASUREMENTS.md`](MEASUREMENTS.md)** — that file is the
record of what is *proven*, and a blank cell in it is an open item here.

## Optics, motion, and power

| Item on hand | Qty | Reported specification |
|--------------|-----|------------------------|
| X-Y galvo kit | 1 | “20Kpps Laser Galvo X-Y Scanning Galvanometer SLA 3D DIY Animation Stage Light”; 400–700 nm mirrors; head rated ±12 V; driver requires ±15 VDC and accepts ±5 V analog input at 0.33 V/° |
| Stereo cameras | 2 | OV9281 global-shutter monochrome camera modules; module/firmware ID, USB mode list, fitted lens focal length, and lens distortion have not yet been recorded |
| Working laser module | 1 | 2.5 W, 450 nm blue, 33 × 70 mm; 12 VDC external “ACC constant-current” drive (seller wording); forced-air cooling; 3-pin TTL switching/PWM power-control interface; anodized-aluminium shell; coated optical-glass collimator |
| Working-laser PSU | 1 | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W |
| Galvo-driver PSU | 1 | **Shipped as part of the galvo kit, with its own mating cables**, so it is the supply the driver was sold to run on and its bipolar output rests on provenance rather than on the marking. Open-frame SMPS, silkscreen **`HT-15V30W V2.0-181206`**, no manufacturer name — 30 W total, no per-rail current stated. Mains enters the end carrying `F1` and the surge device; DC leaves the end carrying the bulk electrolytics and `LED1`. **Both ends are pin headers, not screw terminals**, and the PCB is exposed at both ends of the U-cover, so it cannot be probed live. Rail voltages, per-rail current and **which output pin is `COM`** are still unrecorded |
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
| Level translation | 2 | Marked `SN74AHCT125N`, PDIP-14 quad bus buffer with 3-state outputs — package #1 for the four SPI signals, package #2 for GPIO 18 alone. Sold under the **Editbar** marketplace brand (Zhengzhou Yuanzhang Trading Co., seller SKU **ED1175**); the package logo is not TI's and **no die manufacturer is identified**. `SN…N` is a TI nomenclature, so the marking is a claim by a party that did not make the part |
| Monostable | 1 | **Hitachi HD74HC123P**, DIP-16, date code 9K06 — identified from the package marking 2026-08-03. Marked without the `A` of the `HD74HC123A` datasheet revision obtained |
| AND gate | 1 | SN74HC08N, DIP-14, quad 2-input AND gate |
| Resistors | count not recorded | Carbon film, 1/2 W: 220 kΩ, 10 kΩ and 3 kΩ values. The 1 kΩ is no longer used anywhere — the E-stop sense moved to the 12 V rail and now uses the arm circuit's 10 kΩ/3 kΩ divider (§6) |
| Capacitors | count not recorded | 50 V monolithic ceramic: 1 µF and 100 nF values, plus a 10 µF rail bulk capacitor |
| Zener diode | count not recorded | BZX55C3V3, DO-35 axial, 0.5 W, 3.3 V |
| Arm control | 1 | Lever switch; contact arrangement and DC current rating not recorded |
| Emergency control | 1 | Mushroom button. Contacts read from the fitted blocks 2026-08-09: **1 NO (terminals 3–4) + 1 NC (terminals 1–2).** Generic markings, no manufacturer, no `11-12`/`13-14` numbering, **no visible Annex K direct-opening symbol and no DC rating.** The single NC is now the whole hardware interlock (§3); the NO is left unwired. `ZB2-BE102C` blocks were ordered 2026-08-09 and **refunded** — the architecture was changed instead so that one NC suffices |
| Door interlock | 1 | Only **1** NC contact is needed now (§6a). Positive/direct opening action, DC rating for the driver current, markings and actuator type all not recorded |
| Mains switchgear | **none, by design** | There is no contactor, START button, project-side isolator, OCP or DIN terminal strip. Earlier revisions listed all of these as on hand; none was ever obtained or wired, and §3 was rewritten around that fact. Both supplies plug into an existing wall socket, and its RCD and breaker are the upstream protection |
| Wiring connector | count not recorded | WAGO 221-413 lever connector, 3-conductor, max 4 mm² |

`AHCT` is load-bearing and is **not** interchangeable with `AHC` or `HC`. At
V_CC = 5 V, `AHCT` has TTL input thresholds (V_IH = 2.0 V) and accepts a 3.3 V
Raspberry Pi output with margin; `AHC` and `HC` require 0.7 × V_CC = 3.5 V and
reproduce the exact problem the part exists to solve. All three families share
the same pinout, so a wrong-family substitution is invisible on the assembled
board.

**Marking inspection was the planned defence against that substitution, and for
the devices on hand it no longer works.** They carry a marketplace brand and a
trader's SKU, not a manufacturer's identity, so the `AHCT` letters certify
nothing. The threshold test in `HARDWARE_WIRING.md` §9.4 replaces the marking
check as the decisive one; §9.1 records why even a passing bench threshold does
not fully substitute for a traceable datasheet, and recommends sourcing both
packages from an authorised distributor.

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
| **The design needs 7 × 10 kΩ, not 3** | One arm series resistor, **one E-stop series resistor** (the sense is now 12 V-sourced, §6), **three** laser-path fail-LOW pull-downs — one per node: translator input, 74HC123/74HC08 inputs, and the laser-driver connector (`HARDWARE_WIRING.md` §9.3) — and two chip-select pull-ups (§8). The third pull-down is the only thing holding the driver's own TTL pin LOW when the 74HC08 is unpowered, removed, or its cable is broken; (a) and (b) are both on the translator side and do not reach past the AND gate. | Count and meter-check **7 × 10 kΩ, 1/2 W** out of the bin. Fit each one physically adjacent to the input it protects — a pull-down at the far end of a broken wire protects nothing. |
| Level translation for the SPI and laser paths | Direct 3.3 V drive is not guaranteed into either a 5 V MCP4922 (V_IH = 3.5 V) or the 5 V 74HC123/74HC08 backstop (V_IH = 3.5 V). The translator must be **push-pull** with TTL input thresholds, fast enough for a 50 ns bit period at 20 MHz, and must cover **five** signals — four SPI plus GPIO 18. | **2 × SN74AHCT125N are on hand, without manufacturer traceability** (Editbar / Zhengzhou trading company / SKU ED1175, non-TI logo), so the `AHCT` marking certifies nothing and the planned marking check does not close this row. Open until both devices **pass the input-threshold test of §9.4** — flipping near 1.4–1.5 V, not near 2.5 V, which is what a remarked `AHC`/`HC` die would do — and preferably until both are replaced with parts from an authorised distributor, for the reason in §9.1: this is the only component in the design whose value is a datasheet guarantee rather than a measurable behaviour. Also open until they are fitted with **all three** fail-LOW pull-downs (`HARDWARE_WIRING.md` §9.3), scope-verified per §9.4 — the 20 MHz SPI edges genuinely need one — and bench-verified per §11a, where the ~100 ms one-shot does not. The pin-identical **SN74AHCT126N** with a self-gated active-HIGH `OE` was considered for the laser path and **rejected**: it adds no guarantee that pull-down (b) does not already provide, and it makes the *normal idle* laser TTL a resistive LOW instead of a driven one — reasoning recorded in §9.3. Fit only `AHCT125`. |
| **The E-stop does not latch, and nothing enforces a deliberate restart** | `HARDWARE_WIRING.md` §3 was rewritten: the mushroom's single NC breaks the laser driver's 12 V feed directly, and there is no contactor. Releasing the mushroom **restores that 12 V.** Nothing fires on release, because `SAFE_HALT` is terminal with GPIO 18 LOW and the §9.3 pull-downs holding the TTL line — *unless the control thread had hung with GPIO 18 HIGH*, in which case the laser emits for one 74HC123 one-shot period. Separately, `SAFE_HALT` being terminal means recovery is "restart the software", and restarting with the arm lever still ON walks `INIT → IDLE → ARMED → TRACKING → FIRING` with no deliberate action anywhere. | Two actions, both open. **(1) Implement the `ArmSwitch` LOW→HIGH edge requirement** of `AGENTS.md` §4.8: a process that starts with the lever already up must stay disarmed until the operator cycles it. This is the structural replacement for the contactor's latch, and it is currently a *procedure* instead (§15), which by this project's own standard is not enforcement. **(2) Measure the 74HC123 period and consider shortening it.** In this architecture that number bounds the beam an E-stop *release* can produce after a hung control thread, so ≈220 ms as fitted is worth reducing toward the ~105 ms software bound. |
| **The mushroom's NC contact is unqualified for the duty it now carries** | That single contact is the entire hardware interlock: it switches the laser driver's inrush and running current at **12 V DC**, and GPIO 25 is derived from the same conductor, so a **welded** contact defeats the hardware and the software sense together. The block fitted is generic and unmarked — no manufacturer, no ratings, and **no visible direct-opening (IEC 60947-5-1 Annex K) symbol**, which is precisely the property that forces a welded contact open mechanically. | Measure the laser driver's actual current draw and check it against the block's **DC-13** rating, not its AC rating — direct current has no zero crossing and DC ratings are far lower. Find the Annex K marking on the block or its documentation. If neither can be established, the contact is a single unqualified point of failure, and that must be a recorded decision rather than an oversight. The same duty and the same two questions apply to the arm lever and the door contact, which sit in the same conductor. |
| **Mains terminations are the only mains work, and they are not done** | Both supplies need `L`, `N` and `⏚` landed from a factory-moulded cord. There is no qualified electrician on this build, which is exactly why §3 was rewritten to drive the amount of mains work to near zero rather than to write "have it inspected" beside work that will not be inspected. | Three screw terminations per supply. Brown → `L`, blue → `N`, **green/yellow → the earth terminal and onward to the enclosure, laser chassis and galvo chassis** — PE is never switched and never fused. Keep the terminal covers fitted. Optionally fit a panel IEC C14 inlet with integrated fuse and switch, so the only bare conductors are a short internal run. Verify PE continuity from the plug pin to every chassis before first energisation. |
| **Enclosure door interlock is not installed or verified** | A Class 4 enclosure must terminate beam access when opened. The switch is on hand but nothing has confirmed it has an NC contact with **positive/direct opening action** (IEC 60947-5-1 Annex K), a DC rating for the driver current, and a tool-required actuator — a switch sold as an interlock and a switch that meets Annex K are not the same claim, and the difference is invisible until it fails to open. | **One** NC contact is now sufficient (`HARDWARE_WIRING.md` §6a): with no contactor there is no second circuit to break. Confirm the contact arrangement, the DC rating and the positive-opening marking from the switch's own documentation, then wire it in series with the E-stop and the arm lever on the 12 V laser feed. Prove it by opening the door and watching the disarm. Hardware-only by design — no GPIO and no code change. |
| Lever-switch contacts are unspecified | The switch is intended to interrupt 12 V laser-driver power, not merely provide a logic input. It is now in series with the door interlock's second contact. | Ring out the contacts and verify the DC voltage/current rating against the *measured* laser-module current. DC breaking capacity is the figure that matters here and it is often absent from a switch sold on its AC rating. |
| ±15 VDC galvo supply is unmeasured, and its `COM` is unidentified and unbonded | The supply came inside the galvo kit and mates to the driver on the kit's own cable, so it is the right supply and "is it even bipolar" is settled by provenance. **What the kit cannot have settled is the tie to the rest of this system.** The MCP4922 outputs are 0–5 V referenced to the **Pi's** logic ground; the driver measures `IN+ − IN−` against **its** reference, which is the supply's `COM`. Nothing in the kit ties those two references together, because the kit has never heard of the Pi — and untied, the complementary DAC pair presents an arbitrary common-mode potential to the driver's inputs. The rail voltages are also still blank in `MEASUREMENTS.md`. | Identify `COM` on the 3-pin output header — hold the black probe on one wire and read the other two: `COM` is the wire that gives **+15 V and −15 V**, not +15 V and +30 V. Record all three rails unloaded with the probes on the **wire ends, never on the board** (`HARDWARE_WIRING.md` §17 stage 6), confirm the 30 W covers both driver channels, and bond `COM` into the common signal reference (§13). |
| **Galvo-driver analog input topology and common-mode range are unverified** | This is the assumption with the widest blast radius in the design: `HARDWARE_WIRING.md` §14, `CoordinateMapper`, `DifferentialGalvoDriver` and the choice of two MCP4922s all rest on it. Because the DAC channels are complementary unipolar 0–5 V outputs, the pair presents a permanent **2.5 V common mode**. The vendor's annotated board photo (now recorded in §10) shows the signal inputs as **two separate per-axis connectors at the board edges**, distinct from the central ±15 V inlet — but "±5V singal input" silkscreened beside a connector establishes neither that the inputs are a genuine differential pair, nor that 2.5 V is inside the permitted common-mode range, nor that ±5 V is a *differential* rating rather than per-input-to-ground. If any is wrong, every commanded angle is wrong by a scale or offset error — a silent aiming fault. | **Question 1 closed on the bench, 2026-08-09** (`HARDWARE_WIRING.md` §10): three pins per axis, the centre continuous with the ±15 V centre pin, and **both** outer pins reading finite to that centre — an `IN+`/`GND`/`IN−` difference-amplifier front end. It also identified `COM` with no power applied. **Questions 2 and 3 remain open but are no longer connection risks:** under either reading of "±5 V" the DAC pair stays inside the envelope, so what is at stake is the degrees-per-volt scale, not damage. Obtain the manual if it can be had; otherwise resolve it at `CALIBRATION.md` by commanding a known angle and measuring the actual deflection — with no laser connected. |
| Test-laser electrical details are unknown | A bare 12 mm, 5 mW module may not accept the working laser’s 3-pin TTL interface and therefore may not test the real gating chain. | Record its wavelength, labelled class, supply voltage, current, pinout, and TTL behaviour. Use it for gated alignment only if electrically compatible; otherwise use a suitably classified TTL-controlled alignment module. |
| Working-laser 3-pin pinout and TTL levels are unknown | “TTL/PWM” in a seller listing does not establish pin order, active polarity, input thresholds, or safe power-up state. **Polarity is the critical unknown:** every fail-LOW measure in `HARDWARE_WIRING.md` §9.3 and §11a assumes LOW = off. On an active-LOW input the whole chain inverts and all three pull-downs become a *fire* command — the pull-downs would have to become pull-ups and the AND gate would have to become a different gate. | Obtain the module/driver pinout and confirm the input is **active HIGH** from its own documentation. Verify with the Class 4 optical output disconnected or blocked by an appropriate non-optical test load. Do not establish polarity experimentally with the Class 4 module connected. The project firing path must remain single-level TTL, never PWM. |
| Passive-component quantities/tolerances are still unmeasured | The design needs a timing R/C, per-IC supply decoupling, rail bulk decoupling, two debounce capacitors, two Zener clamps, three laser-path fail-LOW pull-downs, and two chip-select pull-ups. The actual 74HC123 pulse width depends on the fitted R/C values; the vendor is now known (Hitachi, `t_W = R·C`), which puts the nominal at ≈220 ms. | Count and meter-check parts against the running totals in `HARDWARE_WIRING.md` §2: **7 × 10 kΩ**, **8 × 100 nF**, 2 × 3 kΩ, 2 × BZX55C3V3, 1 × 10 µF, 1 × 220 kΩ, 1 × 1 µF. Fit 100 nF at each IC, and measure the one-shot period before connecting either laser. |

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
- **IEC 60947-4-1** — mirror contacts, for detecting a welded contactor pole. Cited for completeness only: this build has no contactor (§3).
- **IEC 60825-1** — laser product safety classification and enclosure
  requirements.
