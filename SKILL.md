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

## Reference Documents (Complete File List)

Quick-scan index of all available reference files. Read as needed:

- `references/control-foc-loops.md`
- `references/algorithm-svpwm-variants.md`
- `references/dq-transform-cordic.md`
- `references/current-sensing-topology.md`
- `references/sensorless-observers.md`
- `references/position-sensors.md`
- `references/stm32g4-foc-hardware.md`
- `references/motor-protection-state-machine.md`
- `references/emergency-protection-halt.md`

## Provide Hardware Acceptance Criteria

Instead of only outputting static C code, you MUST provide explicit physical verification limits or acceptance criteria for the generated design:
- Tell the user exactly what to measure on the oscilloscope (e.g., "Map the estimated `theta_el` and the actual Hall sensor `theta` to DAC channels. They should overlap with minimal phase lag at maximum RPM").
- Highlight physical analog constraints directly related to the code (e.g., "ADC injection MUST be scheduled at the TIM1 TRGO valley, explicitly avoiding ringing from the phase node switching. The minimum pulse width constraint must be respected in 1-shunt implementations").
