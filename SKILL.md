---
name: foc-drive-embedded-c
description: Embedded C development, motor control algorithms, Field Oriented Control (FOC), BLDC six-step commutation, sensing topologies, protection design, and STM32G4 hardware acceleration for digital drives. Use this skill when writing or reviewing firmware for PMSM, BLDC, and related motor-drive systems involving SVPWM, Park/Clarke transforms, current/speed/position cascades, sensorless observers (SMO/PLL), signal filtering, and hardware optimizations such as CORDIC and FMAC when justified by timing, determinism, or control-loop workload.
---

# FOC Drive Embedded C

## Overview

In FOC drive control scenarios, prioritize the output of deterministic, mathematically decoupled, and hardware-accelerated C code tailored for hard real-time execution.

First analyze the motor characteristics, sensing topology (1/2/3 Shunt), mechanical sensor presence, and engineering constraints. Then provide implementation recommendations. Never sacrifice inner current-loop bandwidth, synchronization, or fault response time for coding abstraction.

This skill is optimized primarily for STM32G4-class PMSM and BLDC drives. Adapt its recommendations carefully for other MCU families or for motor types whose observer, flux, or slip models differ materially from PM-machine FOC.

**Convention**: All references use the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling). Do NOT mix with the power-invariant ($\sqrt{2/3}$) form without propagating scaling changes to all dependent equations (SVPWM limits, decoupling, MTPA).

### 1. Analyze System Boundaries Before Writing Code

When user requirements are incomplete, act as a **Pair Programmer** and explicitly ask a short decision-tree question rather than randomly injecting standard libraries or dumping a generic control template.

If critical runtime context is missing, do NOT assume it away. Ask a concise question that directly changes the implementation path, for example:
- "What is your PWM switching frequency?"
- "Are we constrained to an ISR budget below 5 us?"
- "Is this 1-shunt, 2-shunt, 3-shunt, or inline current sensing?"
- "Do you have a real position sensor, or must startup be sensorless?"
- "Is the maximum duty limited by bootstrap refresh requirements or minimum current-sampling windows?"

Key Context Required:
- **MCU model**: (e.g., STM32G4xx)
- **Motor Type**: (e.g., PMSM, BLDC — determines sinusoidal FOC vs. six-step)
- **PWM Frequency**: (e.g., 10kHz, 20kHz, determines your ISR budget)
- **Sensing Topology**: (e.g., Inline, 1-Shunt, 2-Shunt, 3-Shunt? Determines ADC injection setup)
- **Position Sensor**: (e.g., QEP Encoder, Hall, Sensorless SMO, or BEMF zero-crossing?)
- **Command Source**: (e.g., UART, CAN, PWM input, analog throttle, host trajectory stream)
- **Control Mode**: (e.g., torque/current, speed, position, damping/impedance, follow mode)
- **DC Bus Energy Path**: (e.g., battery can absorb regen, active front end can sink power, bench PSU cannot sink, brake resistor present, bus capacitor only)
- **Power Entry Topology**: (e.g., battery, rectified mains, precharged DC link, bench PSU, contactor/precharge relay, fuse strategy)
- **Safety / Recovery Expectations**: (e.g., latched fault, auto-retry allowed, watchdog policy, operator reset required, safe torque off boundary)
- **Mechanical Integration**: (e.g., gearbox, endstop, homing, mechanical brake, backlash/compliance, high-inertia load)
- **Firmware Lifecycle Constraints**: (e.g., field update required, bootloader present, protocol versioning, rollback expectations)

### 2. Parameter Identification

Check if the user knows the Motor Parameters ($R_s, L, \Psi$, Pole Pairs). If unknown, you MUST guide the user through the Auto-Tuning process to identify these statically/dynamically before writing arbitrary PID gains.

## Highest Priority Preferences

- Prefer `float32_t` via FPU for core algorithms unless MTPA tables strictly dictate fixed point.
- Prefer STM32G4 hardware acceleration when it materially improves determinism or ISR cycle margin: typically CORDIC for trigonometric and frame-rotation workloads, and FMAC for repeated filtering or compensator paths when coefficient structure, numeric scaling, and latency budget justify it.
- Prioritize decoupling control (Cross-coupling decoupling) in the dq-frame.
- Make protection (Overcurrent via COMP→TIM_BRK, Overvoltage, Stall) higher priority than speed/position tracking.
- **Hardware Integration Priority**: Enforce a verified timer-triggered ADC synchronization path. Account for dead-time current distortion in all modulation code — during initial bring-up, verify baseline behavior before enabling compensation. Alert the user about PCB Kelvin routing and Low-ESL shunt constraints during 1/2/3 Shunt discussions.
- **Hardware Constraints First**: Prefer physical boundary conditions over canned code recipes. Examples: avoid ADC sampling during PWM-edge ringing, enforce minimum valid current-sampling windows, respect bootstrap refresh duty limits, and preserve comparator/gate-driver shutdown margins.
- **Energy-Flow Awareness**: During deceleration or backdriving, the motor drive can become an energy source into the DC bus. Always identify whether the DC source can absorb regenerative power, whether a brake chopper exists, and what must happen when the bus cannot safely sink returned energy.
- **Power-Stage Survival First**: Check power-entry sequencing, gate-driver UVLO/desat behavior, device SOA, isolation boundaries, and EMI/cabling constraints before assuming the modulation and control law can be used safely on real hardware.
- **Safety Boundaries Matter**: Distinguish nominal control, diagnostic monitoring, and safety action paths. Document what is latched, what may auto-recover, and which assumptions are outside the firmware safety boundary.
- **Supervisory Separation**: Keep host communication, command parsing, and application-level modes decoupled from the hard real-time current loop. Slow interfaces may update targets, but they must not stall, jitter, or directly corrupt the ISR timing path.
- **Explicit Contracts**: When commands or telemetry cross a host boundary, define units, reference frames, mode semantics, and fault/state fields explicitly. Never assume the host and firmware mean the same thing by `speed`, `position`, `torque`, `follow`, or `fault`.
- **Abuse-Case Mindset**: Judge the design by its behavior under sensor dropouts, bus faults, stale commands, bad unit assumptions, and real fault injection, not only by nominal closed-loop performance.
- **Customer-Perceived Quality Matters**: A mathematically stable drive may still fail as a product if it whistles, chatters, or excites resonance. Treat NVH, acoustic behavior, and transition smoothness as real design outputs when relevant.
- **Application Reality Matters**: Do not assume all loads behave like benign bench motors. Compressors, pumps, geared axes, and resonant structures may require operating maps, speed avoidance, gain scheduling, or cross-domain diagnosis.
- **Debuggability Is a Feature**: Prefer designs that can explain themselves under failure. Logging, event snapshots, replayable traces, and explicit bring-up stages are part of product quality, not optional extras.
- **Configuration Discipline Matters**: Treat parameter sets, calibration data, and product variants as governed artifacts. Do not assume the right constants will always be paired with the right hardware.
- **Simulation Boundaries Must Be Honest**: Use SIL and model-based validation to test algorithms, transitions, and assumptions, but do not present them as proof of ADC timing, EMI robustness, gate-driver behavior, thermal safety, or real mechanical/application physics unless they have been hardware-correlated.
- **Measure Before You Conclude**: Prefer trusted instrumentation, well-grounded captures, and symptom-driven domain narrowing over fast but fragile conclusions from a single waveform or log.

## Engineering Checklist (Timing and Verification)

1. ISR Timing Baseline:
   - Capture `DWT_CYCCNT` for full current-loop ISR on target hardware.
   - Confirm baseline: 20kHz FOC must not exceed 10-12µs (hardware + overhead).
   - Document per-block latency: ADC+Clarke, CORDIC/Trig, Current PI, InvPark+SVPWM, TIM update.
   - Track worst-case across operating points (high/low speed, regen, fault recovery).

2. Test and Verification:
   - Add test cases for `t_delay` measurement, `sample_window` calculation, and immediate sampling-phase offset adjustments.
   - Include regression tests for observer lock detection and degraded signal-to-noise behavior.
   - Treat the published timing budget as a measured artifact tied to one firmware build, one hardware revision, and one instrumentation method.

- **Emergency Halt Priority**: When facing unexpected hardware failures or MCU faults, calculating the Safe State (`High-Z` vs `Active Short Circuit`) is your absolute paramount objective. Software recovery algorithms are secondary to preventing equipment fire.
- **Acceleration Philosophy**: Treat CORDIC and FMAC as first-class optimization tools, not dogma. On STM32G4 FOC projects, CORDIC is usually relevant and FMAC is often relevant because current, speed, observer, sensor, and compensator paths frequently include filtering. Use them when they improve real-time behavior without creating unacceptable observability, scaling, or safety-validation risk.
- **Production Code Standard**: Prefer compact, efficient formulations used in production libraries (ST MC SDK, TI MotorWare) over textbook formulations when they are mathematically equivalent. Multiple valid representations exist for algorithms like SVPWM sector determination — verify end-to-end correctness rather than flagging non-textbook forms as bugs.
- **Explicit Override**: If you know a mathematically, computationally, or architecturally superior approach for the target platform (STM32G4) that transcends standard textbook FOC (e.g., Branchless FPU SVPWM sector mappings, zero-wait RAM executions, or a better split between FPU and hardware accelerators), you are ENCOURAGED to propose it. You MUST logically justify the latency vs. math stability trade-offs.

## Core Workflow

When a user requests motor drive code or debugging analysis, use the following default path unless the task is clearly narrower (for example review-only, architecture comparison, or a focused bug hunt).

### Core Bring-Up Path

These are the main steps that define the control architecture and the first-pass implementation:

1. **Information Gathering**: Check if the user specified the Motor Type, Shunt Config, PWM Frequency, Command Source, Control Mode, DC Bus Energy Path, and Power Entry Topology. If any of these materially affect the design and are still unknown, stop and ASK.
2. **Parameter Identification**: Check if motor parameters are known. If not, guide through Auto-Tuning (see `auto-tuning-identification.md`).
3. **Establish Physical Limits**: Before writing any math, read `stm32g4-foc-hardware.md` and `current-sensing-topology.md` to understand the ADC synchronization, dead-time, ringing, Kelvin routing, bootstrap, and PCB routing constraints.
   - If ADC throughput, DMA ownership, or ISR timing margin is a concern, also read `adc-dma-buffering-and-timing.md`.
4. **Select Control Method**: Based on motor type and BEMF profile:
   - **Sinusoidal BEMF (PMSM)** → Sinusoidal FOC (this skill's primary focus)
   - **Trapezoidal BEMF (true BLDC)** → Consider six-step commutation (`bldc-six-step.md`)
   - **"BLDC" with sinusoidal BEMF** → Use sinusoidal FOC despite the label
5. **Select Topology/Sensors**: Based on user context, fetch the specific sensor files (e.g., `sensorless-observers.md` or `position-sensors.md`).
6. **Choose Numeric and Acceleration Strategy**: For transforms, observers, filters, and compensators, decide whether plain FPU code, CORDIC, FMAC, lookup tables, or mixed approaches are best for the MCU budget. Prefer the simplest implementation that still meets timing, determinism, and safety requirements.
7. **Implement Control Loops**: Use `control-foc-loops.md` and `algorithm-svpwm-variants.md` to generate the inner loop C code, enforcing memory placement, timing, and bandwidth separation only as tightly as the actual platform budget requires.
8. **Verify Fault States**: Ensure the design explicitly conforms to the safety states defined in `emergency-protection-halt.md`.
9. **Provide Hardware Acceptance Criteria**: Output the code along with strict physical measurement metrics, real-world analog pain points, and a manual bench-verification plan.

### System Integration Extensions

Use these extensions when the product context requires them. They are not separate from the control design, but they do not need to dominate every narrow task:

1. **Command and Mode Architecture**: If the product is controlled by UART, CAN, PWM input, or a host controller, read `command-and-supervisory-interfaces.md` and separate the command layer from the real-time motor loop. Define mode ownership, timeout behavior, target ramping, and what happens on communication loss.
2. **Telemetry and Diagnostics**: If the design includes a host controller or product interface, read `can-uart-telemetry-and-diagnostics.md` and define command integrity checks, telemetry rates, fault reporting, and communication timeout observability.
3. **Braking and Energy Handling**: If the drive may decelerate high inertia, backdrive, or regenerate, read `braking-and-regeneration.md` and define whether the DC bus can absorb returned energy, whether braking is regenerative or dissipative, and how normal braking differs from emergency halt.
4. **Power Entry and DC-Link Lifecycle**: Read `power-entry-and-dc-link-management.md` when the bus source, precharge, contactor logic, or bus-capacitor lifecycle matters. Define inrush handling, brown-in/brown-out behavior, and what qualifies the bus as ready to enable PWM.
5. **Power-Stage and Gate-Driver Constraints**: Read `gate-driver-and-power-stage-constraints.md` for UVLO, desat, SOA, bootstrap limits, device type trade-offs, and fault containment behavior.
6. **EMC, Isolation, and Cabling Constraints**: Read `emi-emc-isolation-and-cabling.md` when the product includes long motor cables, encoders, isolated interfaces, or aggressive switching edges. Account for common-mode current, shielding, grounding, and isolation assumptions before finalizing firmware behavior.
7. **Production Test and Service Strategy**: Read `production-test-calibration-and-service.md` when the solution must survive manufacturing, field service, or fleet diagnostics. Include self-test, calibration retention, production validation, and service log expectations.
8. **Safety Architecture and Diagnostic Coverage**: Read `safety-architecture-and-diagnostic-coverage.md` when the product requires fault layering, latched/recoverable fault semantics, watchdog policy, or explicit diagnostic coverage boundaries.
9. **Mechanical and Servo Integration Constraints**: Read `mechanical-integration-and-servo-behavior.md` when homing, endstops, brake release, gearbox effects, backlash, resonance, or actuator mechanics influence the control design.
10. **Thermal System Modeling and Derating**: Read `thermal-system-modeling-and-derating.md` when winding, module, heatsink, or enclosure thermal limits materially influence continuous current, overload time, derating law, or sensor placement.
11. **Firmware Lifecycle and Update Strategy**: Read `firmware-lifecycle-and-update-strategy.md` when the product must support bootloaders, field updates, rollback, configuration migration, or host/firmware compatibility policy.
12. **Fault Injection and Abuse Testing**: Read `fault-injection-and-abuse-testing.md` when the product must define how it behaves under bus faults, sensor faults, stale commands, gate-driver faults, or deliberate abuse testing.
13. **Unit Conventions and Control Contracts**: Read `unit-conventions-and-control-contracts.md` when commands, telemetry, formulas, debug signals, or host interfaces risk mixing mechanical/electrical, peak/RMS, torque/current, or sign conventions.
14. **NVH and Acoustic Optimization**: Read `nvh-and-acoustic-optimization.md` when audible noise, torque ripple, vibration, resonance, or transition smoothness matter to the product.
15. **Compliance and Qualification Framing**: Read `compliance-and-qualification.md` when the product must survive qualification-style electrical disturbance, environmental stress, or repeatable revision-to-revision validation.
16. **Motor and Load Characterization**: Read `motor-and-load-characterization.md` when real load inertia, friction, compliance, resonance, or application envelope matter to tuning claims.
17. **Resonance and Speed-Avoidance Strategy**: Read `resonance-identification-and-speed-avoidance.md` when issues occur only in narrow speed or operating bands and the product may need forbidden-speed regions, notch filters, or fast-crossing logic.
18. **Compressor and Refrigeration Applications**: Read `compressor-and-refrigeration-drive-applications.md` when the load depends on suction/discharge pressure, compression ratio, or other refrigeration operating-point variables.
19. **Data Logging and Replay Workflow**: Read `data-logging-replay-and-diagnostics-workflow.md` when the team needs reproducible fault diagnosis, event snapshots, field-return analysis, or structured debug traces.
20. **Commissioning and Bring-Up Playbook**: Read `commissioning-and-bring-up-playbook.md` when the board, motor, or control stack is new and the engineer needs a safe staged validation path before closing loops.
21. **Configuration and Parameter Governance**: Read `configuration-and-parameter-governance.md` when the product has multiple motors, hardware variants, calibration data, or field-updatable parameters that must remain traceable and safe.
22. **Fan and Blower Applications**: Read `fan-and-blower-applications.md` when airflow devices, restriction sensitivity, or low-noise startup behavior define the product.
23. **Pump Applications**: Read `pump-applications.md` when hydraulic load, flow/head variation, startup under load, or fluid-system transients shape the drive behavior.
24. **Servo and Actuator Applications**: Read `servo-actuator-applications.md` when position quality, reversals, hold behavior, endstops, homing, or disturbance rejection dominate the product contract.
25. **SIL and Model-Based Validation Boundaries**: Read `sil-and-model-based-validation-boundaries.md` when the team wants to use simulation or SIL without confusing algorithm verification with proof of real hardware behavior.
26. **Measurement and Instrumentation Best Practices**: Read `measurement-and-instrumentation-best-practices.md` when oscilloscope, probe, DAC, or telemetry setup quality materially affects the conclusion.
27. **Common Symptom to Debug Map**: Read `common-symptom-to-debug-map.md` when the engineer starts from a field symptom and needs to narrow it to sensing, control, power, application, or mechanical domains.

## Reference Documents (Knowledge Base Index)

AI should consult the following domain-specific references when working on the corresponding topics. Fetch files as needed based on the user's request:

When the user starts from a concrete task or symptom and the relevant subset is not obvious, prefer `references/problem-oriented-reading-guide.md` first so the document load stays focused.

- **`references/problem-oriented-reading-guide.md`** -> First-stop navigation by task or symptom: bring-up, ADC timing, startup failure, braking trouble, thermal issues, host-command problems, resonance, SIL boundaries, and application-specific reading paths.

- **`references/auto-tuning-identification.md`** -> **[READ FOR UNKNOWN MOTORS]** Stator Resistance ($R_s$), High-frequency Inductance ($L_d, L_q$), Flux Linkage ($\Psi$), Pole Pairs identification, Zero-Pole Cancellation PI tuning, digital delay bandwidth limits.
- **`references/emergency-protection-halt.md`** -> **[READ FIRST FOR FAULTS]** High-Z coasting vs Active Short Circuit (ASC) decision tree, HardFault handling, STM32G4 BDTR/OISx/OISxN safe-states, Overspeed logic.
- **`references/control-foc-loops.md`** -> Cascaded Loops, PI Feed-Forward with anti-windup, MTPA LUT, Field Weakening voltage-feedback regulator, Bandwidth Rules (10:1 ratio).
- **`references/algorithm-svpwm-variants.md`** -> 7-Segment SVPWM (compact phase-projection and min-max methods), Sector Generation, 5-Segment DPWM constraints, Overmodulation limits, minimum pulse width.
- **`references/dq-transform-cordic.md`** -> Clarke (2-phase KCL optimized), Park, Inverse Park, STM32G4 CORDIC CSR initialization, Async CORDIC ISR pipeline pattern.
- **`references/fmac-filtering-and-compensation.md`** -> When to offload filtering or compensator workloads to STM32G4 FMAC, IIR coefficient loading example, when plain FPU code is better, and what trade-offs must be justified.
- **`references/current-sensing-topology.md`** -> 1-Shunt, 2-Shunt, 3-Shunt timing, Inline sensing, Asymmetric PWM injection, Shunt Resistor ESL, PCB Kelvin connection, RC Anti-aliasing filters, topology selection guide.
- **`references/adc-dma-buffering-and-timing.md`** -> Circular versus ping-pong DMA ownership, ADC sample-packet integrity, compile-time timing guards, and HRTIM as a conditional high-frequency option rather than a default path.
- **`references/command-and-supervisory-interfaces.md`** -> UART/CAN/PWM-input command paths, mode management for torque/speed/position/damping/follow modes, command timeout behavior, target ramping, telemetry, and validation of host-to-motor control behavior.
- **`references/can-uart-telemetry-and-diagnostics.md`** -> Product-grade command framing, telemetry grouping, fault/state reporting, CRC and timeout handling, bus-load limits, and verification of host-visible diagnostics.
- **`references/braking-and-regeneration.md`** -> Normal deceleration versus emergency braking, regenerative versus dissipative energy handling, when the drive behaves as an energy source into the DC bus, brake chopper strategy, source sink-capability constraints, and validation of bus-energy handling.
- **`references/power-entry-and-dc-link-management.md`** -> Power-source assumptions, precharge and inrush control, bus-ready criteria, contactor/relay sequencing, DC-link capacitor stress, brown-in/brown-out behavior, and validation of the power-entry lifecycle.
- **`references/gate-driver-and-power-stage-constraints.md`** -> Gate-driver UVLO/desat behavior, bootstrap edge cases, MOSFET/IPM/SiC/GaN constraints, SOA-aware switching strategy, and fault containment limits at the power-device level.
- **`references/emi-emc-isolation-and-cabling.md`** -> Common-mode current, shielding, grounding, encoder/interface isolation, cable-world dv/dt issues, and practical EMC trade-offs that affect sensing and control reliability.
- **`references/production-test-calibration-and-service.md`** -> Startup self-test, calibration retention, end-of-line checks, service logging, fault injection, and production/field diagnostics expectations.
- **`references/safety-architecture-and-diagnostic-coverage.md`** -> Safety-boundary assumptions, fault layering, watchdog and diagnostic-monitor strategy, latched versus recoverable faults, and expectations for diagnostic coverage and plausibility checks.
- **`references/mechanical-integration-and-servo-behavior.md`** -> Homing, endstops, brake release, gearbox/backlash/compliance effects, resonance-aware servo behavior, and actuator-level validation.
- **`references/thermal-system-modeling-and-derating.md`** -> Winding/module/heatsink thermal paths, sensor-placement validity, thermal time constants, derating law construction, and bench correlation of model versus measured temperature behavior.
- **`references/firmware-lifecycle-and-update-strategy.md`** -> Bootloader/update architecture, rollback policy, protocol/version compatibility, configuration migration, and lifecycle-safe firmware release expectations.
- **`references/fault-injection-and-abuse-testing.md`** -> Deliberate fault injection strategy, electrical/power-stage/sensor/host abuse cases, expected response categories, and acceptance criteria for real failure behavior.
- **`references/unit-conventions-and-control-contracts.md`** -> Mechanical versus electrical frames, peak versus RMS, torque versus current contracts, sign conventions, telemetry naming discipline, and cross-boundary validation rules.
- **`references/nvh-and-acoustic-optimization.md`** -> Audible-noise sources, PWM/dead-time/filtering trade-offs, resonance-aware tuning, and validation of customer-perceived smoothness.
- **`references/compliance-and-qualification.md`** -> Qualification-style disturbance framing, installation and environment assumptions, pass/fail evidence expectations, and firmware-relevant compliance robustness.
- **`references/motor-and-load-characterization.md`** -> Inertia, friction, compliance, resonance, operating-envelope characterization, and why tuning on a free motor is not enough.
- **`references/resonance-identification-and-speed-avoidance.md`** -> Narrow-band vibration diagnosis, electrical/mechanical/order tracking, forbidden-speed and fast-crossing strategies, and resonance mitigation validation.
- **`references/compressor-and-refrigeration-drive-applications.md`** -> Pressure-dependent load behavior, compressor pulsation, correlated pressure/current/vibration logging, and operating-map-based mitigation for refrigeration systems.
- **`references/data-logging-replay-and-diagnostics-workflow.md`** -> Event-triggered snapshots, minimum useful log sets, replay mindset, diagnostic classification, and service-friendly debug workflow expectations.
- **`references/commissioning-and-bring-up-playbook.md`** -> Safe first-power sequence, staged loop enablement, PWM/sensing validation order, common bring-up failure patterns, and what must be proven before moving on.
- **`references/configuration-and-parameter-governance.md`** -> Parameter ownership, variant selection, calibration traceability, corrupted-data handling, and schema/version discipline for real products.
- **`references/fan-and-blower-applications.md`** -> Airflow-related load behavior, startup under restriction, acoustic priorities, and validation of fan/blower-specific operating quality.
- **`references/pump-applications.md`** -> Hydraulic load behavior, startup under head/load, ramp sanity, blocked or abnormal condition handling, and pump-specific validation.
- **`references/servo-actuator-applications.md`** -> Trajectory quality, reversal shock, hold stability, endstop/homing behavior, and actuator-specific validation priorities.
- **`references/sil-and-model-based-validation-boundaries.md`** -> What SIL is good for, what it cannot prove, model-fidelity levels, measured-parameter correlation, and how to distinguish algorithm validation from real physical validation.
- **`references/measurement-and-instrumentation-best-practices.md`** -> Probe selection, grounding pitfalls, timing alignment, DAC/telemetry observability limits, and how to measure drive behavior without fooling yourself.
- **`references/common-symptom-to-debug-map.md`** -> Symptom-driven debug triage, likely fault domains, first measurements to make, and what not to change prematurely.
- **`references/sensorless-observers.md`** -> Sliding Mode Observer (SMO) with sigmoid boundary layer, BEMF extraction, Observer PLL Tracking with correct sign convention, convergence check, High-Frequency Injection (HFI).
- **`references/position-sensors.md`** -> QEP Encoder speed estimation (M/T method), Hall Effect 60-degree angle interpolation and period-based speed, SPI Absolute Encoder delay compensation.
- **`references/stm32g4-foc-hardware.md`** -> Dead-time distortion compensation (parameterized threshold), TIM1 center-aligned PWM initialization, TIM1_TRGO2/CCR4 ADC trigger, Internal OPAMP PGA, COMP→BRK fast trip, ADC dual-regular-simultaneous mode with DMA.
- **`references/motor-protection-state-machine.md`** -> FOC state machine (including TRANSITION state), Startup alignment, Open-Loop V/f ramp, Observer convergence criteria, Bumpless OL→CL transfer, Stall Detection, Catch-spin / flying start, Brake resistor chopper control.
- **`references/bldc-six-step.md`** -> **[READ FOR BLDC]** Trapezoidal commutation table, Hall→sector→gate mapping, STM32G4 TIM1 COM event, Sensorless BEMF zero-crossing with COMP, Advance angle, Six-step vs. FOC decision guide.

## Provide Hardware Acceptance Criteria

Instead of only outputting static C code, you MUST provide explicit physical verification limits or acceptance criteria for the generated design:
- Tell the user exactly what to measure on the oscilloscope (e.g., "Map the estimated `theta_el` and the actual Hall sensor `theta` to DAC channels. They should overlap with minimal phase lag at maximum RPM").
- Highlight physical analog constraints directly related to the code (e.g., "ADC triggering MUST be tied to a verified valid current-sampling instant such as a carefully placed `CCR4/TRGO2` event, explicitly avoiding phase-node ringing. The minimum pulse width constraint must be respected in 1-shunt implementations.").
- State the constraint in physics-first language when possible, not only as a register recipe (e.g., "High di/dt switching causes ringing at the phase node; place the ADC sample at least the measured ringing-settle margin away from PWM edges" or "Bootstrap-powered high-side drivers may lose gate drive if duty is held too close to 100 percent; software must clamp maximum duty accordingly.").
- Explain how the user can verify saturation or windup behavior on real hardware (for example by DAC-exporting the PI integrator state or logging it during commanded saturation).

### Verification Plan

Every substantial code answer should include a short manual verification plan:
- **Manual Verification**: what to probe with scope, DAC, or logged telemetry.
- **Pass Criteria**: what measured behavior is acceptable.
- **Failure Clues**: what waveform, timing, or thermal symptom suggests the code is violating a hardware limit.

## Output Expectations

When providing implementation guidance, prefer this response structure unless the user asks for something narrower:
- State key assumptions that materially affect timing, protection thresholds, or observer validity.
- Distinguish hard safety or physics constraints from platform-specific implementation preferences.
- Explain why CORDIC, FMAC, plain FPU code, or a mixed strategy was chosen.
- Include bench validation guidance, not just static code.
- When code shape is flexible, prefer acceptance criteria over unnecessary code templating: define what the implementation must satisfy on the real board even if multiple software structures are valid.
