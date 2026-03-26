# Gate Driver and Power Stage Constraints (Reference)

## Overview
The controller may compute perfect duty cycles and still destroy the inverter if the gate driver, switches, or switching transitions violate device limits. This document exists to keep the firmware and power stage aligned with what the hardware can actually survive.

## 1. Gate-Driver Critical Constraints

Check whether the gate driver provides:
- undervoltage lockout (UVLO)
- desaturation or overcurrent shutdown
- Miller clamp or strong turn-off
- programmable dead-time or propagation delay variation
- fault latch versus auto-retry behavior
- bootstrap high-side supply or isolated bias rails

Firmware must know which protections are purely hardware, which are latched, and which require explicit recovery sequencing.

## 2. Bootstrap and High-Side Supply Limits

Bootstrap-based high-side drive is common but not free.

Rules:
- sustained high duty can starve bootstrap recharge
- repeated narrow low-side pulses may be insufficient to refresh the bootstrap capacitor
- UVLO events on the high side can create asymmetric or half-valid switching behavior if not handled explicitly

Firmware implications:
- maximum duty and pulse-skipping strategies must consider bootstrap refresh
- if the power stage or modulation strategy approaches bootstrap limits, clamp or reshape operation before driver validity is lost

## 3. Device Technology Matters

Do not treat all switches as interchangeable.

- **Discrete MOSFETs**: flexible, common, sensitive to gate ringing and layout-induced shoot-through
- **IPMs**: often integrate protection and thermal behavior, but impose package-specific limits and fault semantics
- **SiC**: faster edges, stronger EMI pressure, tighter gate-drive and layout discipline
- **GaN**: extremely fast, often less tolerant of sloppy layout or gate-drive assumptions

AI should not suggest aggressive switching or dead-time strategies without considering the actual device family.

## 4. SOA and Thermal Survival

Average current limits are not enough. Check:
- pulse current capability
- switching loss at actual bus voltage and frequency
- repetitive avalanche assumptions
- thermal impedance over the relevant pulse and duty window
- short-circuit withstand time where applicable

Firmware implications:
- overcurrent thresholds and retry logic must respect device survival time, not only motor current needs
- frequency, dead-time, and braking strategy choices change power-device heating materially

## 5. Acceptance Criteria

- **UVLO behavior**: verify that bus sag or bias loss leads to the documented safe output state.
- **Fault path timing**: measure from desat/driver fault or comparator fault to actual gate disable.
- **Bootstrap margin**: at worst-case duty and temperature, verify high-side supply stays above the guaranteed operating threshold.
- **Switching behavior**: confirm gate/drain waveforms do not show uncontrolled ringing, false turn-on, or timing overlap beyond the device margin.
- **Thermal margin**: confirm the chosen PWM frequency and braking behavior do not exceed device thermal limits in representative use.

## 6. Manual Verification Plan

- Probe gate-to-source, switch-node, and DC bus waveforms at representative load and temperature.
- Trigger UVLO-like conditions or bias droop where safe and verify the driver behaves as documented.
- Measure driver fault outputs, break inputs, and actual gate disable timing on the same timebase.
- Re-check at worst-case bus voltage, highest PWM frequency, hottest board condition, and highest braking stress.
