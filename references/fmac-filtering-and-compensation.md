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

## 4. FMAC IIR Filter Example (STM32G4)

When FMAC is justified, here is a reference initialization and usage pattern for a first-order IIR low-pass filter on the current measurement path:

```c
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include "arm_math.h"

#define FOC_ONE (1.0f)
#define FMAC_Q15_SCALE (32767.0f)
#define FMAC_Q15_MIN (-32768)
#define FMAC_Q15_MAX (32767)
#define FMAC_ALPHA_MIN (0.0f)
#define FMAC_ALPHA_MAX (0.99999f)

/**
 * @brief Saturate and convert a float value to Q1.15 fixed-point.
 *        Guards against overflow by clamping to valid Q1.15 range.
 *
 * @param value  Float value to convert (typically in [-1.0, 1.0] for audio/DSP)
 * @return       Saturated Q1.15 representation
 *
 * Safety Rule: This is critical for FMAC. Overflow on coefficient load
 *              can silently corrupt filter behavior with no error indication.
 */
__attribute__((always_inline)) static inline int16_t saturate_to_q15(const float32_t value) {
    float32_t saturated = value * FMAC_Q15_SCALE;
    if (saturated > FMAC_Q15_MAX) {
        saturated = (float32_t)FMAC_Q15_MAX;
    } else if (saturated < FMAC_Q15_MIN) {
        saturated = (float32_t)FMAC_Q15_MIN;
    } else {
        /* Within range — intentionally empty */
    }
    return (int16_t)saturated;
}

/**
 * @brief FMAC IIR filter initialization for current smoothing.
 *
 * Implements a 1st-order IIR: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
 *
 * FMAC operates on Q1.15 fixed-point coefficients.
 * The coefficient array layout for IIR is:
 *   [b0, b1, ..., bN, a1, a2, ..., aM]  (a0 is implicit = 1.0)
 *
 * For a simple LPF with cut-off = alpha:
 *   b0 = alpha,  b1 = 0,  a1 = -(1 - alpha)
 * where alpha = dt / (RC + dt) in [0, 1)
 *
 * CRITICAL: This function validates coefficient ranges. Alpha must be in [0, 0.99999].
 *           Invalid alpha values (< 0 or >= 1) will be rejected with assertion.
 *           This prevents numerical instability and ensures filter convergence.
 */
bool fmac_iir_init(const float32_t alpha) {

    /* SAFETY CHECK: Validate alpha is in valid range [FMAC_ALPHA_MIN, FMAC_ALPHA_MAX] */
    if (__builtin_expect(!!(alpha < FMAC_ALPHA_MIN || alpha > FMAC_ALPHA_MAX), 0)) {
        /* In production, log error and return safely; in debug, assert */
#ifdef DEBUG
        assert(0 && "FMAC alpha out of range [0.0, 0.99999]");
#endif
        return false;  /* Reject invalid coefficient so caller can refuse to enable the path */
    }

    /* Enable FMAC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_FMACEN;

    /* Reset FMAC */
    FMAC->CR = 0u;

    /* Convert float coefficients to Q1.15 with saturation protection */
    const int16_t b0_q15 = saturate_to_q15(alpha);
    const int16_t b1_q15 = 0;
    const int16_t a1_q15 = saturate_to_q15(-(FOC_ONE - alpha));

    /* Configure FMAC for IIR filter */
    /* Load coefficients into FMAC memory via WDATA */
    FMAC->PARAM = (2u << FMAC_PARAM_P_Pos)    /* P = 2 feedforward taps */
                | (1u << FMAC_PARAM_Q_Pos)    /* Q = 1 feedback tap */
                | (8u << FMAC_PARAM_R_Pos)    /* R = buffer size */
                | FMAC_PARAM_FUNC_0;          /* IIR function select */

    /* Preload coefficients: write b0, b1, then a1 */
    FMAC->PARAM |= FMAC_PARAM_START;
    FMAC->WDATA = (uint32_t)(uint16_t)b0_q15;
    FMAC->WDATA = (uint32_t)(uint16_t)b1_q15;
    FMAC->WDATA = (uint32_t)(uint16_t)a1_q15;

    /* FMAC is now ready to process samples */
    return true;
}

/**
 * @brief Push one sample into FMAC and read the filtered output.
 *        Call from the current-loop ISR after ADC reading.
 *
 *        CRITICAL: Input samples must already be scaled into Q1.15 before
 *        narrowing to int16_t. Validate ADC scaling before this stage.
 *        FMAC does not repair a bad engineering-unit conversion.
 *
 * @param raw_q15   Current sample already converted to Q1.15
 * @return          Filtered output (Q1.15, safe for feedback control)
 */
__attribute__((section(".ramfunc"))) __attribute__((always_inline)) static inline int16_t fmac_iir_process(const int16_t raw_q15) {
    /* Write input sample to FMAC (queues for processing) */
    FMAC->WDATA = (uint32_t)(uint16_t)raw_q15;

    /* Read filtered output (hardware blocks until ready, ~1-2 cycles) */
    return (int16_t)FMAC->RDATA;
}
```

**Trade-off vs. FPU software LPF**: The software equivalent is simply `y += alpha * (x - y)` — one multiply and one add, ~2 FPU cycles. FMAC overhead (write `WDATA`, read `RDATA`, Q15 conversion, configuration discipline) may not save anything for a 1st-order filter. FMAC becomes clearly beneficial for **higher-order filters** (FIR with 8+ taps, IIR with 3+ sections) where the arithmetic dominates.

## 5. FMAC Safety Validation Checklist (Critical for Production)

When integrating FMAC into FOC code, verify all of the following:

- **Coefficient Validation**: `saturate_to_q15()` function is called for all floating-point → Q1.15 conversions. No direct unchecked casts.
- **Alpha Range Check**: `alpha` parameter is in [0.0, 0.99999]. Values ≥ 1.0 cause filter instability; negative values reverse polarity unexpectedly.
- **Initialization Success Path**: The caller checks the boolean return from `fmac_iir_init()` and refuses to enable the FMAC path if coefficient validation fails.
- **Input Sample Scaling**: ADC samples are pre-scaled to Q1.15 range **before** calling `fmac_iir_process()`. Unscaled 12-bit ADC values (0–4095) will be interpreted with the wrong magnitude and therefore corrupt the filter and downstream control scaling.
- **Output Interpretation**: FMAC output is Q1.15; if your control law expects full-scale current in Amperes, apply post-scaling: `i_amps = (fmac_output / 32767.0) * full_scale_current_A`.
- **ISR Jitter**: FMAC introduces deterministic latency (~2–4 cycles from write to read). Verify this does not violate your current-loop timing budget.
- **Test Case**: On bench, set a known constant input and verify the output converges to the expected steady-state within a few cycles. A completely wrong input scaling or α value will produce bizarre output patterns immediately visible on a DAC export.

## 6. Recommended AI Guidance

When proposing FMAC in an implementation or review:

1. Name the exact path being offloaded.
2. Explain why that path is timing-critical or repeatedly executed enough to justify FMAC.
3. State the expected benefit in determinism, cycle margin, or throughput.
4. Call out numeric-scaling and saturation assumptions explicitly.
5. Provide a software fallback or verification reference path when feasible.
6. **AI Addition**: Always verify that coefficients are within valid ranges and input data is properly scaled. Suggest bench validation with constant inputs.

## 7. Practical Positioning

For STM32G4 FOC projects, treat CORDIC as the usual first choice for trigonometric and frame-rotation work, and treat FMAC as a first-class option for filtering and compensation workloads that sit on meaningful real-time paths. Both are important, but they solve different bottlenecks.
