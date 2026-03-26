# SIL and Model-Based Validation Boundaries (Reference)

## Overview

Software-In-the-Loop (SIL) can be extremely valuable, but it is also easy to misuse. A clean SIL result does not prove that the real inverter, sensor chain, wiring, thermal path, or mechanical installation will behave the same way.

Use this reference when the team wants to validate control logic, compare algorithm variants, or build model-based confidence without creating false confidence about real hardware.

## 1. What SIL Is Good For

SIL is especially useful for:
- control-law logic
- state-machine sequencing
- anti-windup and saturation behavior
- observer sign and convergence logic
- command conditioning and mode transitions
- unit and reference-frame consistency
- fault sequencing and log generation

These are cases where the code path itself is a large part of the problem.

## 2. What SIL Cannot Prove

SIL should not be treated as proof of:
- ADC trigger placement against real ringing
- op-amp settling or saturation behavior
- dead-time distortion on the actual power stage
- bootstrap droop, gate-driver UVLO, or desat timing
- EMI, ground bounce, cable coupling, or layout-induced measurement corruption
- real thermal propagation and hotspot behavior
- real structural resonance or installation-dependent vibration
- pressure- or process-dependent application physics unless they were explicitly and credibly modeled

Passing SIL is not the same as passing bench validation.

## 3. Model Fidelity Levels

Not all simulations mean the same thing. Distinguish at least these levels:

- **L0: Logic stub**
  - Idealized signal flow with minimal or no plant realism.
  - Good for state-machine and command-path testing.

- **L1: Ideal motor model**
  - Basic electrical/mechanical equations with simplified load.
  - Good for controller structure and sign/unit validation.

- **L2: Non-ideal drive model**
  - Adds approximate saturation, delay, bus limits, or current/voltage limits.
  - Good for better control-law stress testing.

- **L3: Application-aware plant model**
  - Adds representative load behavior, inertia, friction, resonance, or operating-point dependence.
  - Good for application-level algorithm tradeoffs.

- **L4: Correlated model**
  - Parameters and behaviors have been adjusted against measured hardware data.
  - Good for informed engineering comparison, but still not a substitute for final hardware validation.

The team should state which level a given result actually represents.

## 4. Measure Before You Trust the Model

A model becomes more useful only when its parameters come from reality.

Important parameter classes include:
- motor electrical parameters: `Rs`, `Ld`, `Lq`, flux linkage, pole pairs
- mechanical parameters: inertia, friction tendencies, backlash or compliance where relevant
- sensing parameters: offset, gain, latency, plausibility thresholds
- bus and drive behavior: voltage limits, saturation, current limits, modulation limits
- application parameters: pressure ratio, airflow restriction, hydraulic load, resonance band

Each parameter used in SIL should be labeled by source:
- **datasheet estimate**
- **bench measured**
- **identified from runtime data**
- **application assumption**

Do not mix guessed and measured parameters without saying which is which.

## 5. How To Correlate a Model with Real Hardware

The recommended flow is:
1. measure the simplest credible physical parameters first
2. run the model with those values
3. compare simulated and measured responses on a defined test
4. refine only the parts of the model that materially affect the question under study

Useful correlation tests include:
- current step response
- open-loop acceleration profile
- bus-voltage sag or rise under defined conditions
- observer lock/handoff behavior
- commanded ramp versus measured speed response
- narrow-band disturbance or resonance behavior where applicable

Do not keep increasing model complexity if the correlation target itself is not clearly defined.

## 6. Distinguish Algorithm Problems from Physics Problems

SIL is best at isolating algorithm-domain issues such as:
- wrong sign convention
- incorrect mode transition logic
- anti-windup mistakes
- observer math or filtering errors
- unit conversion bugs
- limit and fault sequencing mistakes

Real hardware is still required to resolve physics-domain issues such as:
- invalid current-sampling windows
- EMI-driven sensor corruption
- power-stage timing non-idealities
- mechanical resonance from real installation
- compressor or process load dependence
- thermal drift and hotspot behavior

Do not call a physical problem "fixed in SIL" unless the fix has also been shown on representative hardware.

## 7. Acceptance Criteria for SIL

- **Model-level clarity**: the team can state whether the result is L0, L1, L2, L3, or L4 fidelity.
- **Parameter traceability**: important model parameters have documented sources.
- **Scenario relevance**: the simulation scenario corresponds to a real engineering question, not a generic demo.
- **Algorithm repeatability**: the same inputs reproduce the same control response and event sequencing.
- **Boundary honesty**: SIL results are not presented as proof of hardware safety or physical robustness.

## 8. Manual Verification Plan

- for every SIL scenario, name the corresponding hardware experiment that would later confirm or reject it
- compare at least one measured response against the simulated response before trusting the model for design decisions
- explicitly tag simulation-only conclusions versus hardware-correlated conclusions in reports
- review whether the chosen model fidelity was appropriate for the question being asked

## 9. Guidance for AI Explanations

When AI uses SIL or model-based reasoning, it should say:
- what question SIL is being used to answer
- what model fidelity is assumed
- which parameters are measured versus estimated
- what hardware behavior still remains unproven
- what real experiment should be run next to close the gap

Avoid saying "the system is validated" when only the control logic was validated in an idealized model.
