# Measured Values

This file records **what has actually been measured on this build**. Nothing in
it is copied from a datasheet, a seller listing, or a nominal design value —
those live in [`HARDWARE_PARAMETERS.md`](HARDWARE_PARAMETERS.md) and
[`HARDWARE_WIRING.md`](HARDWARE_WIRING.md).

The distinction is the whole point of the file. `HARDWARE_INVENTORY.md` tracks
what is *present*; this file tracks what is *proven*. **A blank `Measured` cell
is an open no-go item, not a value that is probably fine.**

Two conventions:

- **Fill a row the moment you take the reading**, with the date. A measurement
  remembered a week later is a measurement you will re-take.
- **Where an expected value depends on another measurement, the `Expected`
  column carries the formula, not a nominal.** The 12 V rail measures 12.18 V,
  so the sense junctions target 2.81 V and not the 2.77 V computed from a
  nominal 12.00 V. Writing the formula means the target updates itself as the
  rig gets characterised, instead of quietly disagreeing with the hardware.

---

## 1. Power rails

| Quantity | Expected | Measured | Date | Notes |
|----------|----------|----------|------|-------|
| LRS-50-12 output, **no load** | 12 V ±5 % | **12.18 V** | 2026-08-09 | `SVR` trimmer untouched. This is the reference for every 12 V-derived expectation below |
| LRS-50-12 output, **under laser-driver load** | ≥ 11.5 V | | | Re-measure at §17 stage 7. A significant sag changes both sense-junction targets |
| Galvo supply `+15` → `COM` | ≈ +15 V | | | §17 stage 6, unloaded |
| Galvo supply `COM` → `−15` | ≈ +15 V | | | Red probe on `COM` |
| Galvo supply `+15` → `−15` | ≈ 2 × the two above | | | If this is not about double, the supply is **not bipolar** — no-go for the galvo driver |
| Galvo supply rated current per rail | ≥ both axes together | | | From the nameplate, not from the listing |
| **5 V logic rail, at an MCP4922 `VDD` pin, under load** | 4.75 – 5.25 V | | | §17 stage 3. **This value goes into `dac_reference_voltage`** in `config/system_config.yaml` — the DAC's full-scale is this rail, not 5.000 V |

## 2. GPIO sense networks

Both networks are now identical: 10 kΩ series, 3 kΩ lower leg, 100 nF, BZX55C3V3
with the band at the junction (`HARDWARE_WIRING.md` §5, §6). Measure each
junction **with its GPIO wire disconnected**.

| Quantity | Expected | Measured | Date | Notes |
|----------|----------|----------|------|-------|
| Arm junction, lever ON | `V₁₂ × R_low / (R_ser + R_low)` = **2.81 V** at 12.18 V | | | Recompute from the *metered* resistor values, not the nominals |
| Arm junction, lever OFF | ≈ 0 V | | | Pulled down by the 3 kΩ |
| E-stop junction, mushroom released | same formula = **2.81 V** | | | Tapped after the mushroom, before the lever |
| E-stop junction, mushroom pressed | ≈ 0 V | | | |
| E-stop junction, sense wire disconnected | ≈ 0 V | | | The fail-safe proof: a broken wire and a pressed button must give the same state |
| Arm ratio bench check (5 V on the tap) | `5 × 3 / 13` ≈ 1.15 V | | | Ratio check only — below `V_IH`, so it does not prove "armed" |

**If a junction reads ≈ 0.7 V instead of 2.8 V, the Zener is fitted backwards.**

## 3. Passives, as metered

Colour bands are a claim; the meter is the record.

| Part | Nominal | Measured | Date | Notes |
|------|---------|----------|------|-------|
| Arm series | 10 kΩ | | | |
| E-stop series | 10 kΩ | | | Was a 1 kΩ under the 3.3 V-sourced design; that value is no longer used anywhere |
| Laser fail-LOW (a) translator input | 10 kΩ | | | |
| Laser fail-LOW (b) '123/'08 inputs | 10 kΩ | | | |
| Laser fail-LOW (c) driver connector | 10 kΩ | | | The only cover for an unpowered or removed AND gate |
| CE0 pull-up | 10 kΩ | | | |
| CE1 pull-up | 10 kΩ | | | |
| Arm lower leg | 3 kΩ | | | 3 kΩ, **not** 3.3 kΩ — deliberate, see §5 |
| E-stop lower leg | 3 kΩ | | | |
| One-shot timing R | 220 kΩ | | | On `1Rext/Cext`, **pin 15** |
| One-shot timing C | 1 µF | | | Between pins 14 and 15. Ceramic dielectric unrecorded — can swing the period far more than the ±20 % of R/C tolerance |

## 4. Logic and DAC

| Quantity | Expected | Measured | Date | Notes |
|----------|----------|----------|------|-------|
| DAC X channel A, mid-scale | `V₅ᵣₐᵢₗ / 2` | | | §17 stage 3 |
| DAC X channel B, mid-scale | `V₅ᵣₐᵢₗ / 2` | | | |
| DAC Y channel A, mid-scale | `V₅ᵣₐᵢₗ / 2` | | | |
| DAC Y channel B, mid-scale | `V₅ᵣₐᵢₗ / 2` | | | If these never move while SPI scopes clean, `/LDAC` is floating (§8) |
| `AHCT125` #1 threshold test (§9.4) | output HIGH at 1.99 V in | | | Divider: 10 kΩ from 5 V, 2 × 3 kΩ to GND. Output LOW ⇒ `AHC`/`HC` die ⇒ **no-go** |
| `AHCT125` #2 threshold test (§9.4) | output HIGH at 1.99 V in | | | Both packages must pass; the marking is untraceable and certifies nothing |

## 5. 74HC123 pulse-duration backstop

**In this architecture that period is a safety figure, not a design check.** With
no latch (§3), it bounds the beam an E-stop *release* can produce after a hung
control thread.

| Quantity | Expected | Measured | Date | Notes |
|----------|----------|----------|------|-------|
| One-shot period, assembled | `t_W = R·C` ≈ **220 ms** (Hitachi, no coefficient) | | | Measure in the final configuration; nothing rearranged for the measurement (§11a) |
| Short GPIO 18 pulse → laser TTL | equally short, **not stretched** | | | Stretched ⇒ the AND gate is missing or miswired ⇒ **no-go** |
| Stuck-HIGH GPIO 18 → laser TTL | one period, then stays LOW | | | This is the backstop doing its job |

## 6. Contacts and continuity

| Check | Expected | Result | Date | Notes |
|-------|----------|--------|------|-------|
| Plug `L` → LRS `AC/L` | continuity | | | |
| Plug `N` → LRS `AC/N` | continuity | | | |
| Plug `PE` → LRS `⏚` and chassis | continuity | | | PE is never switched, never fused |
| LRS `L` → `⏚` | **open** | **open** | 2026-08-09 | |
| LRS `N` → `⏚` | **open** | **open** | 2026-08-09 | |
| LRS `L` → `N` | no dead short | **no beep** | 2026-08-09 | A slow rise from the input capacitor is normal |
| Mushroom NC (`1`–`2`), released | closed | | | |
| Mushroom NC (`1`–`2`), pressed | open | | | |
| Mushroom NC — DC-13 rating vs measured driver current | rating ≥ current | | | Carries the whole hardware interlock now. An AC-15 figure is not a DC rating |
| Mushroom NC — Annex K direct-opening symbol | present | | | Arrow inside a circle. The only defence against a welded contact (§3) |
| Lever switch, contact arrangement and DC rating | — | | | Same duty, same conductor |
| Door switch NC, positive opening and DC rating | — | | | One contact is sufficient (§6a) |
| PE continuity: plug pin → enclosure, laser chassis, galvo chassis | continuity | | | Before first energisation |
| Laser driver current draw at 12 V | — | | | Sets the DC rating every contact in the chain must meet |

---

## Where the numbers go

Two measurements are not just records — they are consumed elsewhere:

- **The 5 V logic rail** becomes `dac_reference_voltage` in
  `config/system_config.yaml`. The DAC's full-scale is that rail; leaving 5.000 V
  in the config when the rail sits at 4.92 V is a 1.6 % scale error on every
  commanded angle.
- **The one-shot period** is quoted in `AGENTS.md` §4.1 and §4.8 and in
  `HARDWARE_WIRING.md` §11a. Until it is measured, every statement about the
  pulse-duration backstop rests on an arithmetic prediction.

Everything else in this file exists to close a no-go item in
[`HARDWARE_INVENTORY.md`](HARDWARE_INVENTORY.md).
