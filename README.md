# FOC Drive Embedded C Skill

An AI skill focused on embedded C development for motor control and Field-Oriented Control (FOC) systems, particularly targeting STM32G4xx microcontrollers. This skill provides expert guidance on high-frequency current loops, sensorless observers, space vector modulation, BLDC six-step commutation, and engineering best practices for driving PMSM and BLDC motors, with only partial applicability to ACIM control unless additional machine-specific models are provided.

**Convention**: All references use the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling).

## Core Philosophy

This repository is built as an **AI Control Framework**, not a template library. It dictates:
1. **Interactive Decision Trees**: The AI must ask about your shunt topology, motor type, and PWM frequency before writing code.
2. **Production Code Priority**: Prefer compact, efficient formulations used in production libraries (ST MC SDK, TI MotorWare) over textbook-only approaches when mathematically equivalent.
3. **Override Clauses**: The AI is empowered to propose cutting-edge STM32G4 optimization tricks (CORDIC, FMAC, Branchless ASM) if they outperform standard textbook algorithms.
4. **Hardware Acceptance Criteria**: The AI will instruct you on how to verify its code using oscilloscopes, DACs, and physical constraints.

## Technical References

The `references/` directory contains constraints for the following topics:

**A. Core Control & Algorithms**
- `control-foc-loops.md`: Cascaded PI with anti-windup, MTPA LUT, Field Weakening voltage-feedback regulator.
- `algorithm-svpwm-variants.md`: 7-segment (compact phase-projection & min-max methods), 5-segment DPWM, Overmodulation.
- `dq-transform-cordic.md`: CORDIC-accelerated Park/Clarke, async ISR pipeline, CORDIC CSR initialization.
- `bldc-six-step.md`: Trapezoidal commutation, Hall→gate mapping, TIM1 COM event, sensorless BEMF zero-crossing.

**B. Sensing & Observers**
- `current-sensing-topology.md`: 1/2/3 Shunt timing, 2-shunt reconstruction, Asymmetric PWM, PCB Kelvin routing, topology selection guide.
- `sensorless-observers.md`: SMO with sigmoid boundary layer, PLL tracking (correct sign convention), convergence check, HFI.
- `position-sensors.md`: QEP M/T speed estimation, Hall angle interpolation & period-based speed, SPI delay compensation.

**C. Hardware & System**
- `stm32g4-foc-hardware.md`: TIM1 center-aligned PWM init, ADC dual-simultaneous mode + DMA, OPAMP PGA, COMP→BRK fast trip, Dead-Time distortion compensation.
- `protection-diagnostics.md`: Advanced $I^2t$ thermal overload modeling, dynamic torque limit rollback, protection hierarchy.
- `motor-protection-state-machine.md`: FOC state machine with TRANSITION state, Open-loop V/f ramp, Observer convergence, Bumpless OL→CL transfer, Stall detection, Brake resistor chopper.
- `emergency-protection-halt.md`: HardFault overrides, BDTR/OISx/OISxN safe-states, High-Z vs Active-Short-Circuit (ASC) decision tree, Overspeed.

**D. Generic Drives & Auto-Tuning**
- `auto-tuning-identification.md`: Resistance ($R_s$), Inductance ($L_d/L_q$), BEMF profiling, PI Zero-pole cancellation with digital delay limits, Speed loop tuning.

**E. Acceleration & Filtering**
- `fmac-filtering-and-compensation.md`: FMAC IIR coefficient loading, decision criteria, FPU vs FMAC trade-offs.

## Development Workflow

**Recommended Toolchain**:
- **STM32CubeMX**: Use for initial peripheral configuration (pin mapping, clock tree, timer/ADC setup). Export to CMake project.
- **VSCode + Cortex-Debug + CMake**: Primary development environment. Supports OpenOCD / ST-Link / J-Link debugging.
- **STM32CubeIDE**: Optional alternative, but the CMake-based workflow provides better CI/CD integration and team collaboration.

## Future Roadmap
- Complete the `.vscode` settings and `CMakeLists.txt` open-source build skeleton.
- Define pure Software-In-the-Loop (SIL) mock testing guidelines for FOC current loops.
- Add NVH (Noise, Vibration, Harshness) considerations: PWM frequency selection, spread-spectrum PWM, acoustic noise.
