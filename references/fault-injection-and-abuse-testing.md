# Fault Injection and Abuse Testing (Reference)

## Overview

A product-grade motor drive should be judged not only by how well it runs when everything is healthy, but by how predictably it fails when the plant, sensors, bus, or communication path misbehave.

Use this reference when defining fault handling, production validation, field diagnostics, or service procedures.

## 1. Why Abuse Testing Matters

Nominal closed-loop performance does not prove product robustness.

Deliberate fault injection helps answer:
- which layer detects the fault first
- whether the response is fast enough for the physical hazard
- whether the drive fails to a defined safe state
- whether recovery behavior is deterministic
- whether logs and telemetry explain what happened

## 2. Common Fault Classes

### Electrical Power Faults
- bus overvoltage during braking or backdriving
- bus undervoltage or brownout during torque demand
- precharge failure or unexpected contactor behavior
- supply incapable of sinking regenerative current

### Power-Stage Faults
- overcurrent event
- desaturation or short-circuit indication
- gate-driver UVLO
- bootstrap droop at high duty
- one phase disabled or mis-driven

### Current-Sense Faults
- ADC stuck code
- current-sense offset jump
- gain mismatch
- one channel saturating or drifting
- DMA packet corruption or stale sample ownership failure

### Position / Observer Faults
- Hall bounce or missing transition
- encoder dropout or frozen count
- incorrect direction sign
- observer divergence or false lock
- open-loop to closed-loop handoff failure

### Command / Host Faults
- CAN timeout or bus-off
- UART framing corruption
- impossible mode combination
- stale target stream
- unit mismatch between host and drive

## 3. Abuse Tests Should Match Real Hazards

Do not inject faults randomly. Choose faults that correspond to credible field failures or manufacturing escapes.

Examples:
- disconnect encoder feedback during motion
- force brake command while the bus cannot absorb energy
- hold commanded duty near the bootstrap or minimum-sample-window limit
- introduce current-sense offset drift and confirm plausibility detection
- interrupt host commands and verify timeout policy

## 4. Expected Response Categories

Each injected fault should map to one of a small number of documented behaviors:
- ignore because it is below threshold and plausibly transient
- derate but continue operating
- stop torque and coast
- force a hardware trip or latched fault
- enter a degraded mode with reduced authority

The product should not improvise a response at runtime.

## 5. Acceptance Criteria

- **Detection latency**: the detecting layer responds within the documented time budget for that hazard.
- **Safe-state correctness**: the resulting state matches the product safety definition for that fault class.
- **No hidden oscillation**: the drive does not chatter between run and fault on borderline conditions unless hysteresis is explicitly part of the design.
- **Telemetry usefulness**: logs, fault codes, and state traces identify what failed first and what the drive did next.
- **Recovery determinism**: retry, latch, manual reset, or auto-rearm behavior follows the documented contract.

## 6. Manual Verification Plan

- Inject one fault at a time first, then test realistic combinations such as communication loss during braking or sensor failure during startup.
- Scope the physical reaction, not only the firmware state: gate disable, bus voltage, current collapse, and mechanical coast/brake behavior.
- Log the detection timestamp, fault source, commanded action, and final drive state.
- Repeat the same fault at different speed, load, temperature, and bus conditions.
- Confirm that service or production tools can clearly distinguish injected faults from each other.

## 7. Guidance for AI Explanations

When AI proposes fault handling, it should state:
- which fault is being considered
- which layer detects it first
- what physical hazard is being controlled
- what the expected safe or degraded state is
- how to inject and verify that fault on real hardware

Avoid generic claims like "add protection" without naming the actual injected condition and the measured response.
