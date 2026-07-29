# Reported Hardware Inventory

This file records the hardware reported as physically on hand on 2026-07-29.
It is an inventory record, not a declaration that the assembly is complete or
safe to energize. Vendor-listing values have not yet been independently
measured. Detailed operating parameters and derived limits remain in
[`HARDWARE_PARAMETERS.md`](HARDWARE_PARAMETERS.md); the intended circuit is in
[`HARDWARE_WIRING.md`](HARDWARE_WIRING.md).

## Optics, motion, and power

| Item on hand | Qty | Reported specification |
|--------------|-----|------------------------|
| X-Y galvo kit | 1 | “20Kpps Laser Galvo X-Y Scanning Galvanometer SLA 3D DIY Animation Stage Light”; 400–700 nm mirrors; head rated ±12 V; driver requires ±15 VDC and accepts ±5 V analog input at 0.33 V/° |
| Stereo cameras | 2 | OV9281 global-shutter monochrome camera modules; module/firmware ID, USB mode list, fitted lens focal length, and lens distortion have not yet been recorded |
| Working laser module | 1 | 2.5 W, 450 nm blue, 33 × 70 mm; 12 VDC external “ACC constant-current” drive (seller wording); forced-air cooling; 3-pin TTL switching/PWM power-control interface; anodized-aluminium shell; coated optical-glass collimator |
| Working-laser PSU | 1 | Mean Well LRS-50-12, 12 VDC / 4.2 A / 50 W |
| Alignment/test laser | 1 | 5 mW, 12 mm module; wavelength, supply voltage, pinout, modulation interface, and labelled laser class not yet recorded |

The product listing also advertised higher-power laser variants. They are not
part of this inventory; all project documentation and safety controls refer to
the reported **2.5 W** module only.

## Control and interlock parts

| Item on hand | Qty | Reported specification |
|--------------|-----|------------------------|
| DAC | 2 | MCP4922, DIP-14, dual-channel 12-bit DAC |
| Logic-level module | 1 | Generic 4-channel IIC/I2C bidirectional 3.3 V ↔ 5 V converter module; IC, pull-up values, and maximum edge/data rate not recorded |
| Monostable | 1 | 74HC123, DIP-16; manufacturer and full ordering code not recorded |
| AND gate | 1 | SN74HC08N, DIP-14, quad 2-input AND gate |
| Resistors | count not recorded | Carbon film, 1/2 W: 220 kΩ, 10 kΩ, and **3.3 Ω** values |
| Capacitors | count not recorded | 50 V monolithic ceramic: 1 µF and 100 nF values |
| Zener diode | count not recorded | BZX55C3V3, DO-35 axial, 0.5 W, 3.3 V |
| Arm control | 1 | Lever switch; contact arrangement and DC current rating not recorded |
| Emergency control | 1 | Mushroom button; pole count, NC/NO contact arrangement, and mains rating not recorded |
| Wiring connector | count not recorded | WAGO 221-413 lever connector, 3-conductor, max 4 mm² |

## Inventory-to-design reconciliation

The following gaps are explicit **no-go items**. Do not substitute or bypass a
safety circuit to make the on-hand parts fit.

| Gap | Why it matters | Required resolution before powered assembly |
|-----|----------------|---------------------------------------------|
| The stocked resistor is **3.3 Ω**, not 3.3 kΩ | It is 1000× too small for the GPIO sense dividers. With the documented 10 kΩ upper resistor, the arm input would be only about 4 mV rather than 2.98 V. | Obtain and meter-check **2 × 3.3 kΩ, 1/2 W** resistors. Keep 3.3 Ω parts physically segregated and clearly labelled. |
| No 1 kΩ resistor was reported | The fail-safe E-stop sense circuit in `HARDWARE_WIRING.md` §6 requires one. A 10 kΩ substitution leaves the input below its guaranteed HIGH threshold. | Obtain and meter-check **1 × 1 kΩ, 1/2 W** resistor. |
| Generic I2C level shifter is unspecified | Its topology and edge rate are not known to support 20 MHz push-pull SPI. Also, four channels cannot translate MOSI, SCLK, CE0, CE1, and the laser control path simultaneously. Direct 3.3 V chip-select drive is not guaranteed when an MCP4922 is powered at 5 V. | Use a translator/buffer design qualified for every signal’s voltage, direction, speed, and fail-safe idle state. Scope-test it at the configured SPI rate and verify that the laser-control output defaults LOW through power-up/down and broken-wire cases. |
| Mushroom-button contacts are unspecified | The intended design requires two independent normally-closed contacts, including a pole rated and approved for the mains disconnect. | Verify markings/datasheet for DPST/2×NC operation and applicable mains ratings, or obtain a compliant E-stop assembly. Have mains wiring performed and inspected by a qualified person. |
| Lever-switch contacts are unspecified | The switch is intended to interrupt 12 V laser-driver power, not merely provide a logic input. | Verify its DC voltage/current rating and contact arrangement against measured laser-module current, or obtain a suitably rated switch. |
| ±15 VDC galvo-driver supply was not reported | The galvo driver listing requires ±15 VDC; the 12 V Mean Well supply is for the laser and cannot replace a bipolar ±15 V supply. | Obtain/identify a correctly rated bipolar ±15 VDC supply before testing the galvo driver. |
| Test-laser electrical details are unknown | A bare 12 mm, 5 mW module may not accept the working laser’s 3-pin TTL interface and therefore may not test the real gating chain. | Record its wavelength, labelled class, supply voltage, current, pinout, and TTL behaviour. Use it for gated alignment only if electrically compatible; otherwise use a suitably classified TTL-controlled alignment module. |
| Working-laser 3-pin pinout and TTL levels are unknown | “TTL/PWM” in a seller listing does not establish pin order, active polarity, input thresholds, or safe power-up state. | Obtain the module/driver pinout and verify it with the Class 4 optical output disconnected or blocked by an appropriate non-optical test load. The project firing path must remain single-level TTL, never PWM. |
| Passive-component quantities/tolerances are unknown | The design needs a timing capacitor, per-IC supply decoupling, two debounce capacitors, and two Zener clamps. The actual 74HC123 pulse width depends on the fitted R/C values and vendor. | Count and meter-check parts. Provide at least the quantities in the wiring BOM, fit 100 nF decoupling at each IC, and scope-measure the one-shot period before connecting either laser. |

WAGO 221-413 connectors are three-conductor splicing connectors, not barriers
between unrelated circuits. Use a separate connector for each of Live, Neutral,
PE, and each DC net; install them inside an appropriate enclosure with the
required strain relief and segregation.

## Manufacturer references used for reconciliation

These references support the interface checks above; they do not identify the
manufacturer of an unmarked on-hand part.

- [Microchip MCP4902/4912/4922 data sheet](https://ww1.microchip.com/downloads/en/devicedoc/22250a.pdf)
- [TI CD74HC123 product information](https://www.ti.com/product/CD74HC123)
- [TI SN74HC08 product information](https://www.ti.com/product/SN74HC08)
- [WAGO 221-413 product information](https://www.wago.com/global/installation-terminal-blocks-and-connectors/splicing-connector-with-levers/p/221-413)
