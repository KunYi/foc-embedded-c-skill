# Production Test, Calibration, and Service (Reference)

## Overview
A drive can be mathematically correct and still fail at scale if manufacturing, calibration retention, and service diagnostics are weak. This reference focuses on how a motor-control product is validated and supported beyond the development bench.

## 1. Startup Self-Test

At minimum, consider checks for:
- ADC offset plausibility
- sensor presence and basic range plausibility
- bus-voltage sanity
- gate-driver or fault-line idle health
- stored calibration version and checksum validity

Do not start torque production if the minimum self-test contract is not met.

## 1.1 Manufacturing Self-Test (End-of-Line)

A dedicated manufacturing test mode should execute with minimal human intervention and log a pass/fail result.

Minimum manufacturing checks:
- verify bus voltage and precharge events using an external test fixture
- verify the ADC range and shunt/OPAMP gain with fixed known currents (use calibrated current source)
- verify PWM output and dead-time behavior by measuring 3-phase complementary waveforms
- verify break input and comparator trip path via controlled overcurrent injection (with safe current clamp)
- verify encoder/hall transitions in all sectors (or sensorless angle consistency via open-loop ramp)
- verify firmware mode transitions (STANDBY -> ALIGNMENT -> OPEN_LOOP -> CLOSED_LOOP) for correct state machine sequence
- capture a test trace with time series of `I_d`, `I_q`, `theta_e`, and `V_bus` during a controlled spin run

Include automated result logging and hardware binning tags (GOOD/BAD/REWORK). The production flow must not rely on tribal knowledge.

## 2. Calibration Lifecycle

Define:
- what is calibrated at manufacturing
- what may be recalibrated in the field
- where calibration is stored
- versioning, CRC, and invalid-data fallback behavior

Common items:
- current-sense offset and gain
- bus-voltage scaling
- temperature scaling
- encoder zero or commutation offset

Recommended distinction:
- **Offset calibration**: often performed at startup or service time when torque is disabled and current is known to be zero
- **Gain calibration**: usually established at manufacturing or controlled service using a known current reference or trusted external instrument

Do not silently assume factory resistor tolerances are enough if current accuracy materially affects torque, protection, or fault plausibility thresholds.

## 3. End-of-Line and Production Validation

Typical production checks include:
- bus-power bring-up and precharge verification
- ADC/current-sense offset sanity
- current-sense gain verification against a known current or trusted external reference
- basic PWM output and break-path verification
- sensor direction and plausibility
- no-load spin or limited functional spin where safe
- telemetry and fault reporting sanity

The production flow should not depend on tribal lab knowledge.

## 4. Field Diagnostics and Service

A serviceable product should retain or expose:
- primary fault cause
- warning history
- last-known state and mode
- calibration validity status
- enough telemetry or counters to distinguish sensor, bus, thermal, and communication failures

Service logs should help answer “what failed first?” rather than only “the drive stopped.”

## 5. Fault Injection and Recovery Testing

Before trusting a product, deliberately inject:
- sensor disconnect or implausible values
- communication timeout
- brown-out or undervoltage
- overcurrent trip
- thermal warning and thermal fault

Use MCU internal temperature only as a supplemental diagnostic if needed. Product-grade thermal protection should primarily rely on the temperature channels that actually represent the motor, module, or heatsink thermal state.

Acceptance criteria should define whether the drive latches, retries, derates, or requires operator intervention.

## 6. Acceptance Criteria

- **Self-test coverage**: the documented startup checks run before torque enable and correctly block invalid hardware states.
- **Calibration integrity**: corrupted or version-mismatched calibration does not silently produce incorrect control.
- **EOL repeatability**: production checks identify out-of-family units consistently.
- **Service visibility**: field-readable logs and fault/state data are sufficient to separate likely root causes.
- **Recovery policy**: the drive’s retry, re-arm, and latch behavior matches the documented product policy after injected faults.

## 7. Manual Verification Plan

- Boot with valid and deliberately corrupted calibration records and verify the firmware response.
- Exercise the production test hooks or reduced-spin validation path on representative hardware.
- Confirm that fault logs preserve ordering across repeated fault and reset cycles.
- Verify that a field technician or host tool can read enough diagnostic context to triage bus, sensor, thermal, and communication failures.
