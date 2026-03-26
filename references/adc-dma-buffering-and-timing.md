# ADC DMA Buffering and Timing (Reference)

## Overview
High-rate current control should not waste ISR budget on avoidable register shuffling. On STM32G4-class drives, ADC + DMA is often the difference between a clean deterministic loop and a loop that barely survives worst-case timing.

This reference covers when circular DMA is enough, when double-buffer or ping-pong buffering is worth it, and how to validate that the data path is aligned with the real control deadline.

## 1. Design Goals

The ADC data path should:
- deliver current samples with deterministic timing relative to PWM
- minimize ISR work that does not belong to control math
- preserve sample ordering and channel meaning under sustained operation
- make it easy to prove that no stale or half-updated samples reach the current loop

## 2. Common Buffering Patterns

### A. Single Circular DMA Buffer
- Good for simpler drives where each trigger produces one fixed sample packet.
- Lowest software complexity.
- Best when the ISR consumes data immediately and buffer ownership is unambiguous.

### B. Half-Transfer / Transfer-Complete Split
- Useful when samples arrive in a repeating block and processing can happen at half-buffer granularity.
- Common pattern for continuous acquisition with a stable cadence.
- Requires careful ownership rules so the CPU never processes a region still being written by DMA.

### C. Ping-Pong / Double Buffer
- Best when the control path must process one complete sample set while DMA fills the next.
- Useful for higher PWM rates or when auxiliary channels and decimation logic make the sample packet larger.
- Adds complexity, but gives the cleanest separation between producer and consumer.

## 3. Ownership Rules

The data path should have one clear producer and one clear consumer:

- ADC trigger creates the sample
- DMA writes it into a known buffer region
- ISR or control task consumes only a completed region

Never let the control loop parse a buffer region that may still be under DMA write.

## 4. Timing Rules

- ADC trigger timing still comes first; DMA does not fix a bad sample instant.
- The DMA completion event must arrive early enough that the control loop still meets its ISR budget.
- If double buffering is used, define whether the current loop runs on transfer-complete, half-transfer, or a synchronized control interrupt.

## 5. Compile-Time Guards

Use compile-time checks to block obviously invalid configurations:

```c
_Static_assert(PWM_FREQ_HZ >= CURRENT_LOOP_HZ, "Current loop cannot run faster than PWM trigger rate");
_Static_assert(ADC_SAMPLE_WORDS == 2u || ADC_SAMPLE_WORDS == 4u, "Unexpected ADC DMA packet width");
_Static_assert((DMA_BUFFER_LENGTH % ADC_SAMPLE_WORDS) == 0u, "DMA buffer length must align to complete sample packets");
```

Use compile-time guards for configuration invariants, not for values that must still be bench-validated.

## 6. HRTIM as a Conditional Advanced Option

STM32G4 HRTIM can be valuable when very high switching frequency, finer PWM edge placement, or unusual modulation timing is truly required.

Treat HRTIM as a specialized option, not a default:
- it is most relevant when switching frequency, pulse placement, or power-conversion constraints exceed what the standard advanced timers comfortably support
- it increases integration complexity
- it raises the bar for ADC window design, switching loss control, gate-driver timing, and EMI validation

If the application is not clearly constrained by timer resolution or very high switching frequency, standard TIM1/TIM8 patterns are often the better engineering default.

## 7. Acceptance Criteria

- **Packet integrity**: every DMA packet decodes into the expected channels in the expected order under sustained PWM operation.
- **Ownership correctness**: no buffer region is consumed before DMA has finished writing it.
- **ISR timing margin**: DMA completion, control math, and PWM update all complete within the documented budget at worst-case load.
- **Auxiliary-channel isolation**: bus voltage, temperature, or slow channels do not corrupt or stall the current-loop sample stream.

## 8. Manual Verification Plan

- Toggle a GPIO on DMA half-transfer or transfer-complete and another at control-loop consumption to confirm ordering.
- Inject known DC offsets or static currents and verify DMA packet decoding matches the intended channel map.
- Stress the system at worst-case PWM frequency, full telemetry load, and active protections to confirm the DMA/control path stays deterministic.
- Verify that lost or misordered packets are either detected explicitly or structurally prevented by the ownership design.
