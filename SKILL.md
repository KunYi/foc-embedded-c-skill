---
name: foc-drive-embedded-c
description: Embedded C development, motor control algorithms, Field Oriented Control (FOC), sensing topologies, and protection design specification for STM32G4xx digital drives. Use this skill when writing or reviewing firmware for PMSM, BLDC, induction motors involving SVPWM, Park/Clarke transforms, current/speed/position cascades, sensorless observers (SMO/PLL), and hardware optimizations like CORDIC and FMAC.
---

# FOC Drive Embedded C

## Overview

In FOC drive control scenarios, prioritize the output of deterministic, mathematically decoupled, and hardware-accelerated C code tailored for hard real-time execution.

First analyze the motor characteristics, sensing topology (1/2/3 Shunt), mechanical sensor presence, and engineering constraints. Then provide implementation recommendations. Never sacrifice inner current-loop bandwidth, synchronization, or fault response time for coding abstraction.

### 1. Analyze System Boundaries Before Writing Code

When user requirements are incomplete, act as a **Pair Programmer** and explicitly ask a decision-tree question rather than randomly injecting standard libraries.

Key Context Required:
- **MCU model**: (e.g., STM32G4xx)
- **Motor Type**: (e.g., PMSM, BLDC)
- **PWM Frequency**: (e.g., 10kHz, 20kHz, determines your ISR budget)
- **Sensing Topology**: (e.g., Inline, 1-Shunt, 3-Shunt? Determines ADC injection setup)
- **Position Sensor**: (e.g., QEP Encoder, Hall, or Sensorless SMO?)

## Highest Priority Preferences

- Prefer `float32_t` via FPU for core algorithms unless MTPA tables strictly dictate fixed point.
- Prefer `STM32G4xx` with CORDIC enabled for ALL trigonometric operations (Park, Inverse Park, SVPWM sector logic).
- Prioritize decoupling control (Cross-coupling decoupling) in the dq-frame.
- Make protection (Overcurrent via COMP->TIM_BRK, Overvoltage, Stall) higher priority than speed/position tracking.
- **Hardware Integration Priority**: Enforce strict TIM1_TRGO to ADC synchronization. Never output SVPWM duties without checking for dead-time current distortion. Alert the user about PCB Kelvin routing and Low-ESL shunt constraints during 1/2/3 Shunt discussions.
- **Emergency Halt Priority**: When facing unexpected hardware failures or MCU faults, calculating the Safe State (`High-Z` vs `Active Short Circuit`) is your absolute paramount objective. Software recovery algorithms are secondary to preventing equipment fire.
- **Explicit Override**: If you know a mathematically, computationally, or architecturally superior approach for the target platform (STM32G4) that transcends standard textbook FOC (e.g., Branchless FPU SVPWM sector mappings or zero-wait RAM executions), you are ENCOURAGED to propose it. You MUST logically justify the latency vs. math stability trade-offs.

## Core Workflow (Step-by-Step AI Execution)

When a user requests motor drive code or debugging analysis, you MUST follow this strict sequence:

1. **Information Gathering**: Check if the user specified the Motor Type, Shunt Config, and PWM Frequency. If not, stop and ASK.
1.5 **Parameter Identification**: Check if the user knows the Motor Parameters ($R_s, L, \Psi$, Pole Pairs). If unknown, you MUST guide the user through the Auto-Tuning process to identify these statically/dynamically before writing arbitrary PID gains.
2. **Establish Physical Limits**: Before writing any math, use `view_file` to read `stm32g4-foc-hardware.md` and `current-sensing-topology.md` to understand the ADC synchronization, dead-time, and PCB routing constraints.
3. **Select Topology/Sensors**: Based on user context, fetch the specific sensor files (e.g., `sensorless-observers.md` or `position-sensors.md`).
4. **Implement Control Loops**: Use `control-foc-loops.md` and `algorithm-svpwm-variants.md` to generate the inner loop C code, strictly enforcing `.ramfunc` and the Bandwidth Rule limits.
5. **Verify Fault States**: Ensure the design explicitly conforms to the safety states defined in `emergency-protection-halt.md`.
6. **Provide Hardware Acceptance Criteria**: Output the code along with strict physical measurement metrics (e.g., DAC probe points).

## Reference Documents (Knowledge Base Index)

To prevent Hallucination, AI MUST use `view_file` to read the following domain-specific guidelines before writing code. Fetch files based on the requested topics:

- **`references/auto-tuning-identification.md`** -> **[READ FOR UNKNOWN MOTORS]** Stator Resistance ($R_s$), High-frequency Inductance ($L_d, L_q$), Flux Linkage ($\Psi$), Pole Pairs identification, Zero-Pole Cancellation PI tuning.
- **`references/emergency-protection-halt.md`** -> **[READ FIRST FOR FAULTS]** High-Z coasting vs Active Short Circuit (ASC) decision tree, HardFault handling, STM32G4 BDTR/MOE register safe-states, Overspeed logic.
- **`references/control-foc-loops.md`** -> Cascaded Loops, PI Feed-Forward, MTPA, Field Weakening, Bandwidth Rules (1000Hz->100Hz->10Hz).
- **`references/algorithm-svpwm-variants.md`** -> 7-Segment SVPWM, Sector Generation, 5-Segment DPWM constraints, Overmodulation limits.
- **`references/dq-transform-cordic.md`** -> Clarke, Park, Inverse Park, STM32G4 CORDIC Asynchronous trigonometric calculations.
- **`references/current-sensing-topology.md`** -> 1-Shunt vs 3-Shunt timing, Asymmetric PWM injection, Shunt Resistor ESL, PCB Kelvin connection, RC Anti-aliasing filters.
- **`references/sensorless-observers.md`** -> Sliding Mode Observer (SMO), BEMF estimation, Observer PLL Tracking, High-Frequency Injection (HFI).
- **`references/position-sensors.md`** -> Angle Interpolator, QEP Encoders, Hall Effect 60-degree sector logic, SPI Absolute Encoder delay compensation.
- **`references/stm32g4-foc-hardware.md`** -> Dead-time distortion math/compensation, TIM1_TRGO to ADC precise valley synchronization, Internal OPAMP PGA, COMP->BRK fast trip.
- **`references/motor-protection-state-machine.md`** -> Motor Start-up alignment vectors, Stall Detection, `FOC_STATE` machine, OVP regenerative braking.

## Provide Hardware Acceptance Criteria

Instead of only outputting static C code, you MUST provide explicit physical verification limits or acceptance criteria for the generated design:
- Tell the user exactly what to measure on the oscilloscope (e.g., "Map the estimated `theta_el` and the actual Hall sensor `theta` to DAC channels. They should overlap with minimal phase lag at maximum RPM").
- Highlight physical analog constraints directly related to the code (e.g., "ADC injection MUST be scheduled at the TIM1 TRGO valley, explicitly avoiding ringing from the phase node switching. The minimum pulse width constraint must be respected in 1-shunt implementations").
