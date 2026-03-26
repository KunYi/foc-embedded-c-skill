# Resonance Identification and Speed Avoidance (Reference)

## Overview

Many field problems are not global instability, but narrow speed or frequency regions where the motor, load, structure, or control system couples into an objectionable resonance.

Use this reference when vibration, acoustic harshness, or unstable behavior appears only in certain speed bands or operating windows.

## 1. Typical Resonance Signatures

Resonance-related problems often appear as:
- vibration or noise in a narrow speed range
- large disturbance only near a specific load condition
- stronger behavior at certain pressure ratios, inertia states, or structural mount conditions
- a problem that disappears above or below the affected band

## 2. First Classification

Before changing gains, determine whether the dominant excitation follows:
- mechanical speed
- electrical frequency
- a harmonic or order of rotation
- a structural mode excited by torque ripple, pressure pulsation, or command transitions

This classification decides whether the root cause is mainly electrical, mechanical, fluid-driven, or mixed.

## 3. Practical Mitigation Paths

Common product strategies include:
- forbidden-speed bands
- fast crossing through known resonance regions
- gain scheduling across operating zones
- notch filtering when the frequency target is well understood
- torque-ripple reduction at the electrical source
- mechanical changes to stiffness, damping, mounting, or piping

Do not jump straight to notch filtering unless the problematic frequency is clearly identified and the phase-margin impact is understood.

## 4. Acceptance Criteria

- **Resonance map exists**: the product team can identify the affected region in speed, load, or operating-point terms.
- **Crossing behavior is defined**: the drive either avoids, quickly crosses, or explicitly manages known resonance regions.
- **Mitigation is measurable**: changes reduce the actual vibration or acoustic symptom, not only a control variable.
- **No hidden regression**: the mitigation does not create a worse issue elsewhere in the operating envelope.

## 5. Manual Verification Plan

- Perform speed sweeps while logging current, speed, bus behavior, and vibration or acoustic indicators.
- Repeat the sweep across different load, temperature, and operating conditions.
- Distinguish whether the troublesome band tracks mechanical RPM, electrical frequency, or a known harmonic/order.
- Validate any forbidden-speed or fast-crossing policy under real application transients, not only slow laboratory sweeps.
- If notch or gain scheduling is used, confirm stability margin and behavior outside the targeted band.

## 6. Guidance for AI Explanations

When AI discusses a resonance problem, it should say:
- what quantity likely tracks the resonance
- whether the likely source is electrical, mechanical, fluid/pressure-driven, or mixed
- what mitigation category is appropriate
- how to prove the mitigation on the real product

Avoid treating every narrow speed-band problem as "retune the PI" without identifying what is resonating.
