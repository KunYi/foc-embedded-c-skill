# Mechanical Integration and Servo Behavior (Reference)

## Overview
The motor controller does not operate in isolation. Gearboxes, belts, brakes, backlash, compliance, endstops, and resonance all change what “good control” means.

This reference helps the AI avoid giving mathematically neat but mechanically unsafe advice.

## 1. Mechanical Context Questions

Ask explicitly:
- direct drive or gearbox?
- backlash or elastic coupling?
- high inertia or low inertia?
- hard endstops or soft limits?
- separate mechanical brake present?
- homing required?

The control architecture depends on these answers.

## 2. Homing and Endstops

If the actuator is not absolute at power-up:
- define the homing method
- define the maximum allowed homing torque/current
- define what happens on missed endstop or unexpected obstruction

Do not assume position mode is meaningful before the mechanical reference is established.

## 3. Mechanical Brake Release

If a spring-applied or external brake exists:
- define the order between brake release and torque enable
- define how much holding torque is required before releasing the brake
- define how faults re-engage or coordinate with the brake path

Electrical torque and mechanical brake sequencing must be compatible.

## 4. Backlash, Compliance, and Resonance

High loop gains that look good on a rigid bench load may fail badly with compliance.

Watch for:
- gearbox backlash causing position chatter
- shaft/belt compliance causing oscillation
- resonance excited by aggressive position or damping gains
- follower modes that amplify elastic coupling instead of smoothing it

## 5. Servo-Oriented Validation

- test direction reversals, not just one-way motion
- test settling near the target, not only large moves
- test with realistic load inertia and compliance
- test mode switching when the mechanics are already under load

## 6. Acceptance Criteria

- **Homing behavior**: completes within defined force/current/speed limits and fails safely if reference is not found.
- **Endstop handling**: the drive respects hard and soft limits without repeated hammering or unstable retry.
- **Brake sequencing**: brake release and re-engagement do not create uncontrolled motion or torque spikes.
- **Resonance robustness**: representative load configurations do not excite sustained oscillation in position, damping, or follow modes.

## 7. Manual Verification Plan

- Validate homing and endstop behavior with real mechanics, not only simulated limits.
- Measure motion reversal behavior across backlash and low-speed regions.
- Check brake-release timing against torque command, current, and position movement.
- Test worst-case load inertia and compliant couplings before finalizing outer-loop gains.
