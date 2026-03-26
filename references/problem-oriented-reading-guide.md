# Problem-Oriented Reading Guide (Reference)

## Overview

This guide helps engineers and AI quickly choose the most relevant references for a given problem without reading the entire repository.

Use it as a first-stop index when the symptom or task is clear but the right document set is not.

## 1. If You Are Bringing Up New Hardware

Read in roughly this order:
- `commissioning-and-bring-up-playbook.md`
- `stm32g4-foc-hardware.md`
- `current-sensing-topology.md`
- `adc-dma-buffering-and-timing.md`
- `emergency-protection-halt.md`
- `motor-protection-state-machine.md`

## 2. If Current Sampling or ADC Timing Looks Wrong

Read:
- `current-sensing-topology.md`
- `stm32g4-foc-hardware.md`
- `adc-dma-buffering-and-timing.md`
- `measurement-and-instrumentation-best-practices.md`

## 3. If Startup Fails or Handoff to Closed Loop Is Unstable

Read:
- `motor-protection-state-machine.md`
- `sensorless-observers.md`
- `position-sensors.md`
- `common-symptom-to-debug-map.md`

## 4. If Certain Speeds Vibrate, Buzz, or Sound Bad

Read:
- `nvh-and-acoustic-optimization.md`
- `resonance-identification-and-speed-avoidance.md`
- `motor-and-load-characterization.md`
- `measurement-and-instrumentation-best-practices.md`

For compressor-like products also add:
- `compressor-and-refrigeration-drive-applications.md`

## 5. If Braking or Deceleration Causes Trouble

Read:
- `braking-and-regeneration.md`
- `power-entry-and-dc-link-management.md`
- `emergency-protection-halt.md`
- `protection-diagnostics.md`

## 6. If the Product Uses Host Commands, Modes, or Telemetry

Read:
- `command-and-supervisory-interfaces.md`
- `can-uart-telemetry-and-diagnostics.md`
- `unit-conventions-and-control-contracts.md`
- `data-logging-replay-and-diagnostics-workflow.md`

## 7. If the Problem Seems Thermal or Only Appears Hot

Read:
- `thermal-system-modeling-and-derating.md`
- `protection-diagnostics.md`
- `production-test-calibration-and-service.md`
- `common-symptom-to-debug-map.md`

## 8. If the Issue Is Product Variation, Calibration, or Configuration

Read:
- `configuration-and-parameter-governance.md`
- `production-test-calibration-and-service.md`
- `firmware-lifecycle-and-update-strategy.md`

## 9. If You Need Application-Specific Guidance

Read the matching profile:
- `fan-and-blower-applications.md`
- `pump-applications.md`
- `servo-actuator-applications.md`
- `compressor-and-refrigeration-drive-applications.md`

Then pair it with:
- `motor-and-load-characterization.md`
- `resonance-identification-and-speed-avoidance.md`

## 10. If You Want Simulation or SIL Help Without Fooling Yourself

Read:
- `sil-and-model-based-validation-boundaries.md`
- `auto-tuning-identification.md`
- `measurement-and-instrumentation-best-practices.md`

Use SIL to validate logic and assumptions. Use hardware correlation to validate reality.

## 11. If You Need Fast Fault Triage

Read:
- `common-symptom-to-debug-map.md`
- `fault-injection-and-abuse-testing.md`
- `data-logging-replay-and-diagnostics-workflow.md`
- `measurement-and-instrumentation-best-practices.md`

## 12. Guidance for AI Usage

When AI starts from a concrete problem, it should:
- pick the smallest relevant document set first
- expand only if the first pass points to another domain
- tell the user which references it is relying on and why

Avoid loading large parts of the repository when a symptom clearly points to only one or two domains.
