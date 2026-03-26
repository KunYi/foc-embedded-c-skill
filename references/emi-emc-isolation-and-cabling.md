# EMI, EMC, Isolation, and Cabling (Reference)

## Overview
A drive that works only on a short bench cable is not a finished product. Real installations add long motor leads, encoders, shields, grounds, common-mode current, and regulatory EMI limits.

This document captures the system constraints that often invalidate otherwise-correct control firmware.

## 1. Common-Mode Reality

Fast switching edges drive common-mode current through:
- motor cable capacitance
- heatsink and chassis capacitance
- encoder and sensor cable coupling
- stray capacitance across isolation barriers

Firmware-relevant consequences:
- encoder or sensor corruption
- communication dropouts
- false comparator/ADC events
- unexplained bearing or shaft-current issues in larger systems

## 2. Grounding and Shielding

Ask explicitly:
- where cable shields terminate
- whether chassis and logic ground are bonded and where
- whether encoder/sensor returns share noisy power-stage paths
- whether isolated transceivers or isolated sensor front ends are used

Do not assume a clean digital sensor signal remains clean next to a fast inverter cable.

## 3. Isolation Boundaries

Product-level designs should define:
- which interfaces are isolated and why
- the required creepage/clearance or working-voltage class
- what survives common-mode transients
- how fault signals cross the isolation boundary

Firmware implications:
- watchdog, fault, and telemetry strategies must match the isolation architecture
- an isolated interface that drops out under dv/dt stress should be treated as a system-level risk, not a random software bug

## 4. Cabling and Field Installation

Real cable conditions change behavior:
- long motor leads worsen reflected-wave and dv/dt stress
- unshielded or poorly terminated encoder cables corrupt position feedback
- cable bundles can couple PWM noise into analog and communication lines

AI should warn when a solution that works on a dev board may fail once cable length and installation variation are introduced.

## 5. Acceptance Criteria

- **Sensor robustness**: encoder, Hall, resolver-equivalent, or communication signals remain valid during full-load switching.
- **Noise immunity**: ADC, comparator, and digital interfaces do not false-trigger at representative dv/dt and cable length.
- **Isolation behavior**: isolated interfaces remain functional or fail in the documented safe way during common-mode stress.
- **Cable-world validation**: the drive remains stable with representative motor and sensor cable lengths, not only the bench stub.

## 6. Manual Verification Plan

- Test with representative worst-case cable lengths and routing, not only short bench leads.
- Probe sensor lines, communication lines, and shield/chassis currents during high dv/dt operation.
- Compare behavior with different shield terminations and grounding schemes where product variants allow it.
- Re-run observer, current-sensing, and host-interface validation with the final cable set installed.
