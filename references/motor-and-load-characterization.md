# Motor and Load Characterization (Reference)

## Overview

Control tuning should not treat the plant as "just a motor." Real products behave according to the combined motor, load, inertia, friction, backlash, resonance, and operating envelope.

Use this reference when the product needs robust speed, position, damping, or follow-mode behavior across real application loads.

## 1. Characterization Questions

Before finalizing tuning, define:
- rotor inertia and reflected load inertia
- static friction and viscous friction tendencies
- backlash or compliance
- known resonance bands
- whether the load is constant torque, fan/pump-like, compressor-like, or highly position dependent
- whether the plant changes significantly across temperature or operating point

## 2. Why Characterization Matters

The same current loop can behave very differently when attached to:
- a free-spinning rotor
- a gearbox with backlash
- a pump with load rising with speed
- a compressor with pressure-dependent pulsation
- a compliant or resonant mechanical structure

Tuning on an unloaded bench can therefore be misleading.

## 3. Useful Characterization Outputs

The product team should aim to understand:
- approximate inertia class
- dominant resonance region
- load-versus-speed trend
- low-speed friction or stick-slip behavior
- whether direction reversals or mode transitions excite mechanical issues

This does not require a perfect physics model, but it does require enough plant knowledge to avoid blind loop tuning.

## 4. Acceptance Criteria

- **Plant coverage**: tuning is validated on representative real loads, not only on a no-load bench motor.
- **Transition sanity**: speed changes, reversals, and follow-mode transitions do not excite uncontrolled oscillation or shock beyond the documented product target.
- **Resonance awareness**: known resonance bands are identified and handled explicitly rather than discovered accidentally in the field.
- **Operating-envelope awareness**: tuning and limits reflect the real load range the product will see.

## 5. Manual Verification Plan

- Run controlled sweeps across speed and load to identify unstable or noisy regions.
- Compare behavior with and without the real mechanical load attached.
- Log command, speed, current, and vibration or displacement indicators during reversals and ramps.
- Test at the operating extremes that customers actually reach, not only convenient lab points.
- Revisit characterization whenever the gearbox, mounting, rotor, or load hardware changes.

## 6. Guidance for AI Explanations

When AI explains tuning or loop behavior, it should say:
- what kind of load model is assumed
- whether the behavior is bench-only or representative of the real application
- what mechanical effect could invalidate the tuning
- how the plant should be characterized before making stronger claims

Avoid treating "motor tuning" as complete when the load has not yet been characterized.
