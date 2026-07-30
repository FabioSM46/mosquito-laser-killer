# Wiring Diagrams

Excalidraw sources plus PNG renders, numbered in **reading order**: orient with
the overview, then work inward.

| Drawing | Covers | Document sections |
|---------|--------|-------------------|
| `1 - system-overview` | The whole signal chain end to end, plus what each interlock actually removes | all |
| `2 - mains-power-distribution` | Mains inlet, isolator/OCP/RCD, K1 contactor, the latching control circuit, DC branches | [§3](../HARDWARE_WIRING.md#3-mains-power-safety-contactor--e-stop), [§4](../HARDWARE_WIRING.md#4-dc-power-distribution) |
| `3 - gpio-sense-circuits` | Arm and E-stop sense dividers, Zener polarity, door interlock | [§5](../HARDWARE_WIRING.md#5-arming-switch--gpio-24-sensing-circuit), [§6](../HARDWARE_WIRING.md#6-e-stop-gpio-circuit), [§6a](../HARDWARE_WIRING.md#6a-enclosure-door-interlock) |
| `4 - spi-level-translation` | AHCT125 #1, both MCP4922s with full pin assignment, galvo driver inputs | [§8](../HARDWARE_WIRING.md#8-spi-bus--mcp4922-dac-wiring), [§9](../HARDWARE_WIRING.md#9-logic-level-translation), [§10](../HARDWARE_WIRING.md#10-galvo-driver-connections) |
| `5 - laser-ttl-safety-chain` | AHCT125 #2, the 74HC123 one-shot, the AND gate, all three fail-LOW pull-downs | [§9.3](../HARDWARE_WIRING.md#93-the-laser-path-must-fail-low-at-every-stage-not-just-the-first), [§11](../HARDWARE_WIRING.md#11-laser-module-wiring), [§11a](../HARDWARE_WIRING.md#11a-laser-ttl-pulse-duration-backstop-74hc123) |

## Rules

- **[`HARDWARE_WIRING.md`](../HARDWARE_WIRING.md) is the source of truth.** These
  drawings follow it. Where they disagree, the document is correct and the drawing
  is stale — fix the drawing, and never build from a drawing that contradicts it.
- **Reading order is not build order.** Building in this sequence would put live
  mains in the work area at drawing 2, while the circuits are still on a
  breadboard. The risk-ordered build sequence is
  [§17](../HARDWARE_WIRING.md#17-build--bring-up-order); stages 0–5 need no mains
  at all.
- **A filled black dot is a connection. A crossing without a dot is not.** This
  matters most in drawing 3: an earlier revision left the 3.3 kΩ divider legs
  unconnected, and built that way the GPIO pin still reads HIGH — so the fault is
  invisible until the Zener fails.
- Pin numbers are PDIP throughout, and are annotated in the drawings only so they
  can be checked against the manufacturer's datasheet — not so they can be
  trusted without checking.

## Editing

The sources are plain `.excalidraw` JSON. Either open them at
[excalidraw.com](https://excalidraw.com), or drive the canvas server:

```bash
npx -y mcp-excalidraw-server import "docs/diagrams/3 - gpio-sense-circuits.excalidraw" --replace
# edit, then:
npx -y mcp-excalidraw-server export --out "docs/diagrams/3 - gpio-sense-circuits.excalidraw"
npx -y mcp-excalidraw-server screenshot --out "docs/diagrams/3 - gpio-sense-circuits.png"
```

Re-export the PNG whenever the source changes — a stale render next to a Class 4
laser is worse than no render.
