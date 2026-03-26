# Commissioning and Bring-Up Playbook (Reference)

## Overview

Early bring-up is where many drives are damaged, mischaracterized, or debugged in the wrong order. A product-grade workflow should define what to verify first, what not to enable too early, and which measurements prove the system is ready for the next step.

Use this reference for first power-up, new board revisions, new motor pairings, or major control-stack changes.

## 1. Bring-Up Order Matters

Do not start by closing loops just because the firmware builds.

A safe commissioning sequence is usually:
- verify power-entry behavior
- verify gate-driver and fault path behavior
- verify sensing offsets and timing
- verify PWM polarity and dead-time assumptions
- verify command and telemetry path sanity
- only then enable controlled open-loop behavior
- only then close current, speed, or position loops

## 2. Phase 1: Unpowered and Low-Risk Checks

Before energizing the motor:
- confirm shunt polarity and gain assumptions
- confirm phase labeling and sensor wiring
- confirm gate-driver enable/reset polarity
- confirm fault inputs and break path polarity
- confirm parameter set matches the actual hardware variant

## 3. Phase 2: Controlled Power-Up

At first power:
- verify precharge and bus-ready logic
- confirm the bus voltage is what firmware believes it is
- verify no unexpected PWM activity at reset or idle
- verify faulted state leaves outputs in the intended safe condition

## 4. Phase 3: PWM and Sensing Validation

Before closing any loop:
- scope the PWM outputs and dead-time
- verify ADC trigger placement against ringing and settling
- verify current-sense offsets at zero torque
- confirm sample ordering and scaling
- confirm bus voltage and slow-channel readings are sane

## 5. Phase 4: Low-Risk Motion

Before full closed-loop control:
- start with limited current and a conservative bus voltage if possible
- use open-loop alignment or low-authority motion first
- confirm direction correctness
- confirm current response polarity
- confirm the observer or position sensor agrees with actual rotation

## 6. Phase 5: Close Loops in Order

Typical order:
- current loop
- observer/speed estimate validation
- speed loop
- position/follow/damping modes

Do not hide a broken current loop under a speed loop or application mode.

## 7. Common Bring-Up Failure Patterns

Watch for:
- wrong phase order
- wrong current-sign convention
- ADC trigger inside ringing
- stale or mis-packed DMA samples
- sensor offset mistaken for torque
- host command units not matching firmware expectation
- thermal and protection thresholds copied from another product

## 8. Acceptance Criteria

- **Safe idle**: the drive powers up without unexpected torque or PWM assertion.
- **Sensing validity**: zero-current offsets, gain, and timing are verified before loop closure.
- **Direction correctness**: commanded direction, measured direction, and estimated direction agree.
- **Loop staging correctness**: each loop is validated before the next outer loop depends on it.
- **Fault containment**: intentional fault tests work before normal operation is trusted.

## 9. Manual Verification Plan

- Record a bring-up checklist for each new board or product variant rather than relying on memory.
- Save representative oscilloscope captures for PWM timing, ADC timing, and first controlled motion.
- Log the first successful open-loop motion and first successful current-loop closure as reference baselines.
- Re-run critical bring-up steps whenever the gate driver, sensing chain, motor wiring, or timer/ADC configuration changes.

## 10. Guidance for AI Explanations

When AI helps with bring-up, it should say:
- what stage the engineer is currently in
- what must be proven before moving to the next stage
- what measurements are required at that stage
- what common failure modes are most likely at that point

Avoid skipping directly to "here is the final loop code" when basic bring-up evidence is still missing.
