# FOC Drive Embedded C Skill

An AI skill focused on embedded C development for motor control and Field-Oriented Control (FOC) systems, particularly targeting STM32G4xx microcontrollers. This skill provides expert guidance on high-frequency current loops, sensorless observers, space vector modulation, BLDC six-step commutation, and engineering best practices for driving PMSM and BLDC motors, with only partial applicability to ACIM control unless additional machine-specific models are provided.

**Convention**: All references use the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling).

**GitHub**: <https://github.com/KunYi/foc-embedded-c-skill>

## Core Philosophy

This repository is built as an **AI Control Framework**, not a template library. It dictates:
1. **Interactive Decision Trees**: The AI must ask about the runtime constraints that materially change the design path before writing code, such as shunt topology, motor type, PWM frequency, command source, control mode, and DC-bus energy handling assumptions.
2. **Production Code Priority**: Prefer compact, efficient formulations used in production libraries (ST MC SDK, TI MotorWare) over textbook-only approaches when mathematically equivalent.
3. **Override Clauses**: The AI is empowered to propose cutting-edge STM32G4 optimization tricks (CORDIC, FMAC, Branchless ASM) if they outperform standard textbook algorithms.
4. **Hardware Acceptance Criteria**: The AI will instruct you on how to verify its code using oscilloscopes, DACs, and physical constraints.

## Technical References

The `references/` directory contains constraints for the following topics:

- `problem-oriented-reading-guide.md`: quick navigation by task or symptom, helping engineers and AI choose the right subset of references first.

**A. Core Control & Algorithms**
- `control-foc-loops.md`: Cascaded PI with anti-windup, MTPA LUT, Field Weakening voltage-feedback regulator.
- `algorithm-svpwm-variants.md`: 7-segment (compact phase-projection & min-max methods), 5-segment DPWM, Overmodulation.
- `dq-transform-cordic.md`: CORDIC-accelerated Park/Clarke, async ISR pipeline, CORDIC CSR initialization.
- `bldc-six-step.md`: Trapezoidal commutation, Hall→gate mapping, TIM1 COM event, sensorless BEMF zero-crossing.

**B. Sensing & Observers**
- `current-sensing-topology.md`: 1/2/3 Shunt timing, 2-shunt reconstruction, Asymmetric PWM, PCB Kelvin routing, topology selection guide.
- `adc-dma-buffering-and-timing.md`: ADC ownership models, circular vs ping-pong DMA, ISR timing margin, and conditional HRTIM usage.
- `command-and-supervisory-interfaces.md`: UART/CAN/PWM-input command paths, mode management, target conditioning, timeout handling, and host-to-motor validation.
- `can-uart-telemetry-and-diagnostics.md`: command framing, telemetry design, fault reporting, communication timeout behavior, and diagnostics validation.
- `sensorless-observers.md`: SMO with sigmoid boundary layer, PLL tracking (correct sign convention), convergence check, HFI.
- `position-sensors.md`: QEP M/T speed estimation, Hall angle interpolation & period-based speed, SPI delay compensation.

**C. Hardware & System**
- `stm32g4-foc-hardware.md`: TIM1 center-aligned PWM init, ADC dual-simultaneous mode + DMA, OPAMP PGA, COMP→BRK fast trip, Dead-Time distortion compensation.
- `protection-diagnostics.md`: Advanced $I^2t$ thermal overload modeling, dynamic torque limit rollback, protection hierarchy.
- `motor-protection-state-machine.md`: FOC state machine with TRANSITION state, Open-loop V/f ramp, Observer convergence, Bumpless OL→CL transfer, Stall detection, Brake resistor chopper.
- `emergency-protection-halt.md`: HardFault overrides, BDTR/OISx/OISxN safe-states, High-Z vs Active-Short-Circuit (ASC) decision tree, Overspeed.
- `braking-and-regeneration.md`: normal braking vs emergency halt, regenerative vs dissipative braking, DC bus sink capability, brake chopper strategy, and energy-handling verification.
- `power-entry-and-dc-link-management.md`: precharge, inrush, contactor/relay sequencing, bus-ready logic, DC-link stress, and brown-in/brown-out handling.
- `gate-driver-and-power-stage-constraints.md`: UVLO, desat, bootstrap limits, MOSFET/IPM/SiC/GaN constraints, and SOA-aware fault handling.
- `emi-emc-isolation-and-cabling.md`: common-mode current, shielding, grounding, isolation assumptions, encoder/cable robustness, and EMI validation.
- `thermal-system-modeling-and-derating.md`: winding/module/heatsink thermal paths, sensor placement validity, derating law design, and thermal validation.
- `production-test-calibration-and-service.md`: self-test, calibration retention, end-of-line checks, field diagnostics, and service logging.
- `safety-architecture-and-diagnostic-coverage.md`: safety boundaries, latched vs recoverable faults, watchdog layering, plausibility checks, and diagnostic coverage framing.
- `mechanical-integration-and-servo-behavior.md`: homing, endstops, mechanical brake release, gearbox/backlash/compliance effects, and resonance-aware servo integration.
- `firmware-lifecycle-and-update-strategy.md`: bootloader/update strategy, rollback, protocol compatibility, configuration migration, and release safety.
- `fault-injection-and-abuse-testing.md`: deliberate fault injection, abuse cases, expected response categories, and failure-behavior validation.
- `unit-conventions-and-control-contracts.md`: mechanical vs electrical units, RMS vs peak, torque/current contracts, sign conventions, and telemetry naming discipline.
- `nvh-and-acoustic-optimization.md`: acoustic/noise sources, PWM and dead-time trade-offs, resonance awareness, and validation of smoothness.
- `compliance-and-qualification.md`: qualification-style disturbance framing, repeatable validation evidence, and firmware-relevant product robustness expectations.
- `motor-and-load-characterization.md`: inertia/friction/compliance characterization, resonance awareness, and why tuning on a free motor is not enough.
- `resonance-identification-and-speed-avoidance.md`: narrow-band vibration diagnosis, forbidden-speed strategy, and resonance mitigation validation.
- `compressor-and-refrigeration-drive-applications.md`: pressure-dependent compressor loads, correlated pressure/current/vibration analysis, and operating-map-based mitigation.
- `data-logging-replay-and-diagnostics-workflow.md`: event snapshots, replay mindset, diagnostic classification, and field-friendly debug workflow.
- `commissioning-and-bring-up-playbook.md`: staged first-power validation, safe loop-enablement order, and common bring-up failure patterns.
- `configuration-and-parameter-governance.md`: parameter ownership, variant discipline, NVM corruption handling, and calibration traceability.
- `fan-and-blower-applications.md`: airflow-device behavior, restriction sensitivity, startup quality, and acoustic priorities.
- `pump-applications.md`: hydraulic-load behavior, startup under head/load, and pump-specific protection and ramp concerns.
- `servo-actuator-applications.md`: trajectory quality, reversal behavior, hold stability, homing/endstop handling, and actuator-focused validation.
- `sil-and-model-based-validation-boundaries.md`: what SIL can prove, what it cannot, model-fidelity levels, measured-parameter correlation, and simulation boundary honesty.
- `measurement-and-instrumentation-best-practices.md`: probe choice, grounding pitfalls, bandwidth honesty, DAC/telemetry limits, and how to measure drive behavior correctly.
- `common-symptom-to-debug-map.md`: symptom-driven triage, likely fault domains, first measurements to make, and what not to change too early.

**D. Generic Drives & Auto-Tuning**
- `auto-tuning-identification.md`: Resistance ($R_s$), Inductance ($L_d/L_q$), BEMF profiling, PI Zero-pole cancellation with digital delay limits, Speed loop tuning.

**E. Acceleration & Filtering**
- `fmac-filtering-and-compensation.md`: FMAC IIR coefficient loading, decision criteria, FPU vs FMAC trade-offs.

## Development Workflow

**Recommended Toolchain**:
- **STM32CubeMX**: Use for initial peripheral configuration (pin mapping, clock tree, timer/ADC setup). Export to CMake project.
- **VSCode + Cortex-Debug + CMake**: Primary development environment. Supports OpenOCD / ST-Link / J-Link debugging.
- **STM32CubeIDE**: Optional alternative, but the CMake-based workflow provides better CI/CD integration and team collaboration.

## Basic Usage With AI Models

Most modern large language models and agent frameworks can work with a repository-level `SKILL.md` pattern. This repository is organized so the model can read the top-level `SKILL.md` first, then pull in only the relevant files from `references/` based on the task.

**Recommended usage pattern**:
1. Place this repository where your AI tool can access it, or install it into the tool's shared skills/knowledge directory.
2. Ask the model to use the repository as a `SKILL.md`-style engineering skill for FOC, STM32G4 motor control, or embedded drive firmware review.
3. Let the model read `SKILL.md` first, then selectively load the relevant `references/*.md` files for the topic at hand.
4. When asking for code or architecture help, provide concrete project constraints such as motor type, bus voltage, shunt topology, PWM frequency, sensing method, protection requirements, and MCU target.
5. Ask the model to return both implementation guidance and bench-validation steps so the result stays grounded in real hardware behavior.

**Typical prompt examples**:
- "Use this `SKILL.md` repo to review my STM32G4 PMSM FOC current loop implementation."
- "Using the attached skill, design a sensorless startup strategy for a 2-shunt inverter on STM32G4."
- "Use this motor-control skill to explain the Hall interpolation math and how to validate it on hardware."
- "Review this FOC protection flow against product-grade fault handling and bench verification requirements."

**What the AI should do well with this repo**:
- Explain control theory, embedded implementation trade-offs, and STM32G4 hardware constraints in a way that is useful for pair engineering.
- Distinguish hard safety constraints from platform-specific preferences.
- Propose product-grade validation steps using oscilloscope captures, logged telemetry, comparator trips, ADC timing checks, and thermal/protection tests.
- Use `CORDIC`, `FMAC`, observers, sensing, and modulation references as engineering options, not blind rules.
- Catch unit, sign, and frame mismatches before they become product bugs.
- Explain likely abuse cases and how to validate failure behavior, not only nominal operation.
- Treat acoustic smoothness and resonance avoidance as real product requirements when the application cares about them.
- Recognize when tuning claims depend on the real load, not only the bare motor.
- Help the team reason about qualification-style disturbance behavior and repeatable product evidence.
- Diagnose coupled application problems such as pressure-dependent compressor vibration instead of assuming every symptom is a generic PI issue.
- Provide a usable debug workflow with event-triggered logging and replay-oriented diagnosis, not just advice to print more variables.
- Guide staged commissioning so engineers know what must be proven before they energize the next control layer.
- Treat parameter packs, calibration data, and hardware variants as governed product assets rather than loose constants.
- Adapt recommendations to common application families such as fans, pumps, and servo actuators instead of treating every load the same way.
- Use SIL or model-based reasoning responsibly, without overstating simulation as proof of real hardware timing, EMI, thermal, or mechanical behavior.
- Tell engineers how to measure the system without being misled by bad probe setup or mismatched observability channels.
- Start from real symptoms and narrow fault domains systematically instead of immediately rewriting the control law.

## License

This project is released under the `MIT` License.

Copyright (c) 2026 KUNYI CHEN `<kunyi.chen@gmail.com>`

See [LICENSE](./LICENSE) for the full text.

## Future Roadmap
- Complete the `.vscode` settings and `CMakeLists.txt` open-source build skeleton.
- Define pure Software-In-the-Loop (SIL) mock testing guidelines for FOC current loops.
- Add cross-vendor portability guidance for non-STM32G4 platforms while preserving the same physics-first validation mindset.
