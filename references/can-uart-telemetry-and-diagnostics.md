# CAN, UART, Telemetry, and Diagnostics (Reference)

## Overview
A motor-control product is not complete when the control loop works once on the bench. It also needs a robust way to command targets, report state, surface faults, and support debugging without destabilizing the drive.

This document covers the practical design of communication and telemetry around a FOC drive, especially when the motor is controlled by UART, CAN, or a host-side application.

## 1. Communication Design Goals

The communication plane should:
- deliver commands without blocking or jittering the real-time loop
- expose enough telemetry to validate behavior and diagnose faults
- detect stale, corrupted, or implausible commands
- provide deterministic fallback behavior on link failure

Communication exists to support the drive, not to own its timing-critical execution.

## 2. Command Framing and Data Integrity

### UART
- Prefer framed packets with length, command ID, payload, and checksum/CRC.
- Do not rely on newline-delimited debug text for product control.
- Use DMA or interrupt-driven RX into a ring buffer rather than polling in time-critical code.

### CAN
- Define explicit message ownership: command frames, telemetry frames, heartbeat, and fault/status frames.
- For multi-axis systems, define whether timestamps or sequence counters are required to detect stale coordination data.
- If using a higher-level protocol such as CANopen, keep the mapping to motor-control units explicit.

### Common Integrity Rules
- Every command path should define freshness, plausibility, and authority.
- A syntactically valid packet is not necessarily a safe packet.
- Out-of-range targets, impossible mode combinations, or inconsistent unit flags must be rejected before they reach the control loops.

## 3. Telemetry Design

Recommended telemetry groups:

- **Fast control telemetry**: `Iq_ref`, `Id_ref`, measured current, bus voltage, electrical angle, electrical speed, modulation index
- **Motion telemetry**: mechanical position, mechanical speed, active control mode, conditioned target
- **Protection telemetry**: fault code, warning code, timeout status, derating state, thermal estimator state
- **Communication telemetry**: packet age, heartbeat age, CRC error count, bus-off or framing-error counters

Do not stream everything at the highest rate. Decimate telemetry by purpose:
- fast enough to diagnose control behavior
- slow enough not to starve CPU, DMA, or bus bandwidth

### Telemetry Unit Contract
Every externally visible field should have an explicit and stable definition:

- quantity name
- engineering unit
- reference frame (`mechanical`, `electrical`, `bus`, `phase`, `dq`, etc.)
- scaling and signedness
- whether the signal is raw, filtered, estimated, commanded, or conditioned

Examples that must be disambiguated:
- `speed`: mechanical rad/s, electrical rad/s, mechanical RPM, or filtered estimate?
- `position`: encoder counts, mechanical radians, wrapped electrical angle, or multi-turn position?
- `torque_cmd`: physical N*m request, normalized percentage, or `Iq_ref` in amperes?

## 4. Fault and State Reporting

Define fault reporting as a product interface, not an afterthought.

At minimum, report:
- active mode
- ready / armed / enabled / faulted state
- current fault cause
- whether torque output is currently inhibited
- whether communication timeout fallback is active

A host should not have to infer motor safety state indirectly from current or speed alone.

### Recommended State / Fault Schema
Standardize host-visible fields so diagnostics do not depend on tribal knowledge:

- `state`: `boot`, `standby`, `armed`, `alignment`, `open_loop_start`, `closed_loop`, `derating`, `fault`
- `active_mode`: `torque`, `speed`, `position`, `damping`, `follow`
- `warning_flags`: bitfield for degraded-but-running conditions such as command timeout fallback, thermal derating, sensor quality degraded, bus low margin
- `fault_flags`: latched fault bitfield for shutdown conditions such as overcurrent, overvoltage, undervoltage, overspeed, sensor fault, communication fault, thermal trip
- `fault_code_primary`: the dominant shutdown reason presented to the host
- `fault_latched`: whether restart is blocked until explicit clear/re-arm
- `torque_inhibit`: whether the inverter is currently prevented from driving torque

If multiple faults occur, define priority rules so the reported primary fault is deterministic.

## 5. Physical and Product Constraints

- **Logging overhead**: high-rate telemetry can consume meaningful CPU, DMA bandwidth, RAM, and bus bandwidth. The telemetry path must not starve the control path.
- **Timestamp meaning**: for control diagnosis, timestamps must be monotonic and tied to a known timebase.
- **Unit discipline**: host software must know whether a value is mechanical or electrical, filtered or raw, commanded or conditioned.
- **Schema stability**: state/fault field meanings should remain stable across firmware revisions unless the host protocol version changes with them.
- **Fault determinism**: communication failure must lead to a deterministic drive state, not an ambiguous "last value wins" behavior.
- **Debug isolation**: verbose debug print paths must never execute in the high-rate ISR.

## 6. Acceptance Criteria

- **CRC / checksum robustness**: corrupted frames are rejected without producing unsafe motion.
- **Timeout behavior**: communication timeout transitions the drive into the documented fallback state within the specified time.
- **Telemetry fidelity**: logged telemetry matches known injected references, commanded steps, or bench stimuli closely enough to debug the drive.
- **Unit/schema fidelity**: each host-visible field decodes to the documented unit, sign, and reference frame. A host decoder built from the public contract should not need hidden firmware knowledge.
- **Bus loading**: maximum expected telemetry + command traffic does not break control timing or exceed bus utilization targets.
- **Fault visibility**: when an overcurrent, overspeed, timeout, or thermal event occurs, the host can identify the cause and active safety state without ambiguity.

## 7. Manual Verification Plan

- Inject malformed UART packets and verify they are rejected cleanly.
- Force CAN heartbeat loss or bus-off and verify the mode manager applies the documented fallback.
- Compare streamed telemetry against oscilloscope or DAC-observed internal signals to confirm scaling and timestamp alignment.
- Verify that host-visible state, warning, and fault fields change in the documented order during arm, run, timeout, derate, and shutdown scenarios.
- Run worst-case telemetry load while the motor operates near its control bandwidth limit and confirm loop timing remains stable.
- Verify that fault reports remain latched, time-ordered, and understandable after repeated fault/recover cycles.
