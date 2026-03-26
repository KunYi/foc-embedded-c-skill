# FOC Drive Embedded C Skill

An AI skill focused on embedded C development for motor control and Field-Oriented Control (FOC) systems, particularly targeting STM32G4xx microcontrollers. This skill provides expert guidance on high-frequency current loops, sensorless observers, space vector modulation, and engineering best practices for driving PMSM, BLDC, and ACIM motors.

## Core Philosophy

This repository is built as an **AI Control Framework**, not a template library. It dictates:
1. **Interactive Decision Trees**: The AI must ask about your shunt topology and PWM frequency before writing code.
2. **Override Clauses**: The AI is empowered to propose cutting-edge STM32G4 optimization tricks (CORDIC, FMAC, Branchless ASM) if they outperform standard textbook algorithms.
3. **Hardware Acceptance Criteria**: The AI will instruct you on how to verify its code using oscilloscopes, DACs, and physical constraints.

## Technical References

The `references/` directory contains constraints for the following topics:

**A. Core Control & Algorithms**
- `control-foc-loops.md`: Cascaded PI, MTPA, Field Weakening.
- `algorithm-svpwm-variants.md`: 7-segment, 5-segment DPWM, Overmodulation.
- `dq-transform-cordic.md`: CORDIC-accelerated Park/Clarke and rotation.

**B. Sensing & Observers**
- `current-sensing-topology.md`: 1/2/3 Shunt timing, Asymmetric PWM, PCB Kelvin routing, RC filtering limits.
- `sensorless-observers.md`: SMO, High-frequency injection (HFI).
- `position-sensors.md`: QEP, Hall Effect, Absolute Encoders.

**C. Hardware & System**
- `stm32g4-foc-hardware.md`: OPAMP, COMP->BRK, TIM1_TRGO ADC Sync, Dead-Time distortion compensation.
- `motor-protection-state-machine.md`: Startup, Open-loop alignment, Stall detection.
- `emergency-protection-halt.md`: HardFault overrides, High-Z vs Active-Short-Circuit (ASC), Overspeed.

## Future Roadmap
- Complete the `.vscode`, `CMakeLists.txt` open-source build skeleton.
- Define pure Software-In-the-Loop (SIL) mock testing guidelines for FOC current loops.

*Draft initialized dynamically via Agentic AI Scaffold.*
