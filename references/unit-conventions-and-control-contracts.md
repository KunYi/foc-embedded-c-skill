# Unit Conventions and Control Contracts (Reference)

## Overview

Many expensive motor-drive bugs are not caused by unstable control laws, but by inconsistent units, sign conventions, or hidden assumptions between firmware, host software, and validation tooling.

Use this reference whenever values cross module boundaries, host boundaries, or math-domain boundaries.

## 1. Conventions Must Be Explicit

Do not let the product silently guess:
- electrical angle vs mechanical angle
- electrical speed vs mechanical speed
- torque command vs current command
- RMS vs peak
- phase voltage/current vs line quantities
- radians per second vs RPM
- dq sign convention and reference-frame orientation

## 2. Host-Firmware Contract

Every externally commanded or reported quantity should define:
- name
- unit
- sign convention
- reference frame
- scaling and numeric type
- filtered or raw meaning
- saturation behavior

Examples:
- `speed_cmd_radps_mech`
- `theta_elec_rad`
- `iq_ref_a_peak`
- `torque_limit_nm`

Prefer explicit names over overloaded generic fields like `speed`, `angle`, or `current`.

## 3. Internal Math Contract

Inside the control stack, define:
- which Clarke/Park convention is used
- whether current and voltage values are peak or RMS quantities
- whether `Iq > 0` means motoring in the positive mechanical direction
- how negative speed, reverse torque, and reverse electrical angle are represented

If one layer uses amplitude-invariant Clarke and another assumes power-invariant scaling, the whole loop can become numerically wrong while still looking superficially reasonable.

## 4. Validation and Tooling Contract

Logs, DAC exports, and oscilloscope annotations should state:
- unit
- sign
- raw vs filtered
- mechanical vs electrical frame
- commanded vs conditioned vs measured meaning

The debug path should never require tribal knowledge to decode a plotted variable correctly.

## 5. Acceptance Criteria

- **Naming clarity**: critical command and telemetry fields are explicit enough that a new engineer does not need hidden assumptions to interpret them.
- **Frame consistency**: no mixing of mechanical and electrical quantities without an explicit conversion.
- **Magnitude consistency**: RMS, peak, phase, and line quantities are not silently interchanged.
- **Sign consistency**: forward motoring, reverse motoring, and regenerative braking all follow the documented sign rules.
- **Cross-tool agreement**: host tools, firmware logs, and scope/DAC debug channels agree on the same units and meanings.

## 6. Manual Verification Plan

- Inject positive and negative commands in every supported mode and verify sign behavior end to end.
- Compare host-reported units against firmware-internal values for at least one known operating point.
- Verify pole-pair conversion explicitly when translating between mechanical and electrical quantities.
- Review a set of recorded telemetry channels and ensure a new engineer can decode them from names and documentation alone.
- Deliberately test near zero crossing and direction reversal where sign or frame mistakes are easiest to miss.

## 7. Guidance for AI Explanations

When AI explains a formula or interface, it should always say:
- what frame the variable lives in
- whether the quantity is mechanical or electrical
- whether it is RMS or peak when that distinction matters
- what sign means motoring versus generating

Avoid phrases like "set speed to 1000" or "compare angle and speed" without explicit units and frames.
