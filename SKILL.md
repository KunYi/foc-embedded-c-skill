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

### 2. Parameter Identification

Check if the user knows the Motor Parameters ($R_s, L, \Psi$, Pole Pairs). If unknown, you MUST guide the user through the Auto-Tuning process to identify these statically/dynamically before writing arbitrary PID gains.

## Highest Priority Preferences

- Prefer `float32_t` via FPU for core algorithms unless MTPA tables strictly dictate fixed point.
- Prefer STM32G4 hardware acceleration when it materially improves determinism or ISR cycle margin: typically CORDIC for trigonometric and frame-rotation workloads, and FMAC for repeated filtering or compensator paths when coefficient structure, numeric scaling, and latency budget justify it.
- Prioritize decoupling control (Cross-coupling decoupling) in the dq-frame.
- Make protection (Overcurrent via COMP→TIM_BRK, Overvoltage, Stall) higher priority than speed/position tracking.
- **Hardware Integration Priority**: Enforce strict TIM1_TRGO to ADC synchronization. Account for dead-time current distortion in all modulation code — during initial bring-up, verify baseline behavior before enabling compensation. Alert the user about PCB Kelvin routing and Low-ESL shunt constraints during 1/2/3 Shunt discussions.
- **Hardware Constraints First**: Prefer physical boundary conditions over canned code recipes. Examples: avoid ADC sampling during PWM-edge ringing, enforce minimum valid current-sampling windows, respect bootstrap refresh duty limits, and preserve comparator/gate-driver shutdown margins.
- **Energy-Flow Awareness**: During deceleration or backdriving, the motor drive can become an energy source into the DC bus. Always identify whether the DC source can absorb regenerative power, whether a brake chopper exists, and what must happen when the bus cannot safely sink returned energy.
- **Supervisory Separation**: Keep host communication, command parsing, and application-level modes decoupled from the hard real-time current loop. Slow interfaces may update targets, but they must not stall, jitter, or directly corrupt the ISR timing path.
- **Explicit Contracts**: When commands or telemetry cross a host boundary, define units, reference frames, mode semantics, and fault/state fields explicitly. Never assume the host and firmware mean the same thing by `speed`, `position`, `torque`, `follow`, or `fault`.
- **Emergency Halt Priority**: When facing unexpected hardware failures or MCU faults, calculating the Safe State (`High-Z` vs `Active Short Circuit`) is your absolute paramount objective. Software recovery algorithms are secondary to preventing equipment fire.
- **Acceleration Philosophy**: Treat CORDIC and FMAC as first-class optimization tools, not dogma. On STM32G4 FOC projects, CORDIC is usually relevant and FMAC is often relevant because current, speed, observer, sensor, and compensator paths frequently include filtering. Use them when they improve real-time behavior without creating unacceptable observability, scaling, or safety-validation risk.
- **Production Code Standard**: Prefer compact, efficient formulations used in production libraries (ST MC SDK, TI MotorWare) over textbook formulations when they are mathematically equivalent. Multiple valid representations exist for algorithms like SVPWM sector determination — verify end-to-end correctness rather than flagging non-textbook forms as bugs.
- **Explicit Override**: If you know a mathematically, computationally, or architecturally superior approach for the target platform (STM32G4) that transcends standard textbook FOC (e.g., Branchless FPU SVPWM sector mappings, zero-wait RAM executions, or a better split between FPU and hardware accelerators), you are ENCOURAGED to propose it. You MUST logically justify the latency vs. math stability trade-offs.

## Core Workflow (Step-by-Step AI Execution)

When a user requests motor drive code or debugging analysis, use the following default sequence unless the task is clearly narrower (for example review-only, architecture comparison, or a focused bug hunt):

1. **Information Gathering**: Check if the user specified the Motor Type, Shunt Config, and PWM Frequency. If not, stop and ASK.
2. **Parameter Identification**: Check if motor parameters are known. If not, guide through Auto-Tuning (see `auto-tuning-identification.md`).
3. **Establish Physical Limits**: Before writing any math, read `stm32g4-foc-hardware.md` and `current-sensing-topology.md` to understand the ADC synchronization, dead-time, ringing, Kelvin routing, bootstrap, and PCB routing constraints.
4. **Select Control Method**: Based on motor type and BEMF profile:
   - **Sinusoidal BEMF (PMSM)** → Sinusoidal FOC (this skill's primary focus)
   - **Trapezoidal BEMF (true BLDC)** → Consider six-step commutation (`bldc-six-step.md`)
   - **"BLDC" with sinusoidal BEMF** → Use sinusoidal FOC despite the label
5. **Select Topology/Sensors**: Based on user context, fetch the specific sensor files (e.g., `sensorless-observers.md` or `position-sensors.md`).
6. **Define Command and Mode Architecture**: If the product is controlled by UART, CAN, PWM input, or a host controller, read `command-and-supervisory-interfaces.md` and separate the command layer from the real-time motor loop. Define mode ownership, timeout behavior, target ramping, and what happens on communication loss.
7. **Define Telemetry and Diagnostics**: If the design includes a host controller or product interface, read `can-uart-telemetry-and-diagnostics.md` and define command integrity checks, telemetry rates, fault reporting, and communication timeout observability.
8. **Define Braking and Energy Handling**: If the drive may decelerate high inertia, backdrive, or regenerate, read `braking-and-regeneration.md` and define whether the DC bus can absorb returned energy, whether braking is regenerative or dissipative, and how normal braking differs from emergency halt.
9. **Choose Numeric and Acceleration Strategy**: For transforms, observers, filters, and compensators, decide whether plain FPU code, CORDIC, FMAC, lookup tables, or mixed approaches are best for the MCU budget. Prefer the simplest implementation that still meets timing, determinism, and safety requirements.
10. **Implement Control Loops**: Use `control-foc-loops.md` and `algorithm-svpwm-variants.md` to generate the inner loop C code, enforcing memory placement, timing, and bandwidth separation only as tightly as the actual platform budget requires.
11. **Verify Fault States**: Ensure the design explicitly conforms to the safety states defined in `emergency-protection-halt.md`.
12. **Provide Hardware Acceptance Criteria**: Output the code along with strict physical measurement metrics, real-world analog pain points, and a manual bench-verification plan.

## Reference Documents (Knowledge Base Index)

AI should consult the following domain-specific references when working on the corresponding topics. Fetch files as needed based on the user's request:

- **`references/auto-tuning-identification.md`** -> **[READ FOR UNKNOWN MOTORS]** Stator Resistance ($R_s$), High-frequency Inductance ($L_d, L_q$), Flux Linkage ($\Psi$), Pole Pairs identification, Zero-Pole Cancellation PI tuning, digital delay bandwidth limits.
- **`references/emergency-protection-halt.md`** -> **[READ FIRST FOR FAULTS]** High-Z coasting vs Active Short Circuit (ASC) decision tree, HardFault handling, STM32G4 BDTR/OISx/OISxN safe-states, Overspeed logic.
- **`references/control-foc-loops.md`** -> Cascaded Loops, PI Feed-Forward with anti-windup, MTPA LUT, Field Weakening voltage-feedback regulator, Bandwidth Rules (10:1 ratio).
- **`references/algorithm-svpwm-variants.md`** -> 7-Segment SVPWM (compact phase-projection and min-max methods), Sector Generation, 5-Segment DPWM constraints, Overmodulation limits, minimum pulse width.
- **`references/dq-transform-cordic.md`** -> Clarke (2-phase KCL optimized), Park, Inverse Park, STM32G4 CORDIC CSR initialization, Async CORDIC ISR pipeline pattern.
- **`references/fmac-filtering-and-compensation.md`** -> When to offload filtering or compensator workloads to STM32G4 FMAC, IIR coefficient loading example, when plain FPU code is better, and what trade-offs must be justified.
- **`references/current-sensing-topology.md`** -> 1-Shunt, 2-Shunt, 3-Shunt timing, Inline sensing, Asymmetric PWM injection, Shunt Resistor ESL, PCB Kelvin connection, RC Anti-aliasing filters, topology selection guide.
- **`references/command-and-supervisory-interfaces.md`** -> UART/CAN/PWM-input command paths, mode management for torque/speed/position/damping/follow modes, command timeout behavior, target ramping, telemetry, and validation of host-to-motor control behavior.
- **`references/can-uart-telemetry-and-diagnostics.md`** -> Product-grade command framing, telemetry grouping, fault/state reporting, CRC and timeout handling, bus-load limits, and verification of host-visible diagnostics.
- **`references/braking-and-regeneration.md`** -> Normal deceleration versus emergency braking, regenerative versus dissipative energy handling, when the drive behaves as an energy source into the DC bus, brake chopper strategy, source sink-capability constraints, and validation of bus-energy handling.
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
