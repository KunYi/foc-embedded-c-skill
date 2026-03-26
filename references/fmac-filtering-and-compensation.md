# FMAC for Filtering and Compensation (Reference)

## Overview
STM32G4 FMAC is a hardware accelerator for repeated filtering and compensator math. In FOC systems it is often relevant, but it is not automatically the best answer for every loop. Use it when it improves deterministic throughput or cycle margin without making numeric scaling, debug visibility, or safety validation materially worse.

## 1. Where FMAC Commonly Helps in FOC

- **Current-measurement conditioning**: Repeated low-pass or smoothing filters on sampled currents, especially when the ADC path is noisy and the same filter executes every PWM cycle.
- **Speed estimation smoothing**: Hall, QEP, or sensorless speed estimates often benefit from deterministic filtering to reduce torque ripple and outer-loop noise.
- **Observer paths**: SMO, PLL, or BEMF estimation chains frequently need low-pass filtering or compensator-like stages that execute at fixed cadence.
- **Compensators and resonant shaping**: Notch filters, lead-lag blocks, and digital compensator sections can be reasonable FMAC candidates when coefficients are stable and loop timing is tight.

## 2. When Plain FPU Code May Be Better

- **Low-rate outer loops**: If the speed or position loop runs slowly and the MCU has wide cycle margin, simple FPU code may be easier to inspect and tune.
- **Bring-up and commissioning**: During early bench work, transparent software filters are often easier to validate than hardware-offloaded paths.
- **Rapidly changing structures**: If the filter form or coefficients change frequently at runtime, FMAC setup overhead and scaling complexity may outweigh the benefit.
- **Small filter workload**: If the implementation is only one or two lightweight sections, FMAC may add complexity without meaningful savings.

## 3. Decision Criteria

Before recommending FMAC, explicitly evaluate:

- **ISR cycle budget**: Is filter or compensator math a real contributor to loop latency?
- **Execution rate**: The higher the repetition rate, the more attractive FMAC becomes.
- **Numeric format and scaling**: Can the chosen coefficients, input range, and state evolution be represented safely with acceptable quantization and saturation margin?
- **Data movement cost**: Consider the cost of moving samples and reading results, not just the arithmetic itself.
- **Debug and observability cost**: If debugging the path becomes much harder, justify why the cycle savings are worth it.
- **Fallback path**: Prefer designs where an FPU implementation can be kept as a reference for verification.

## 4. Recommended AI Guidance

When proposing FMAC in an implementation or review:

1. Name the exact path being offloaded.
2. Explain why that path is timing-critical or repeatedly executed enough to justify FMAC.
3. State the expected benefit in determinism, cycle margin, or throughput.
4. Call out numeric-scaling and saturation assumptions.
5. Provide a software fallback or verification reference path when feasible.

## 5. Practical Positioning

For STM32G4 FOC projects, treat CORDIC as the usual first choice for trigonometric and frame-rotation work, and treat FMAC as a first-class option for filtering and compensation workloads that sit on meaningful real-time paths. Both are important, but they solve different bottlenecks.
