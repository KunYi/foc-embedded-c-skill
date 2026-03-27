# DQ Transform & CORDIC Integration (Reference)

## Overview
Field-Oriented Control involves continuous rotations of the reference frame using Clarke and Park transforms. The `sin(theta)` and `cos(theta)` operations mathematically dominate the cycle count.

The implementation examples below show common STM32G4 production patterns. Use CORDIC when it improves deterministic timing or frees enough CPU budget to matter; plain FPU code may remain preferable during bring-up, low-rate control paths, or when simplicity outweighs the saved cycles.

**Convention**: This skill uses the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling). All downstream documents (SVPWM, circle limitation, decoupling) are consistent with this convention. Do NOT mix with the power-invariant ($\sqrt{2/3}$) form without propagating the scaling change to every dependent equation.

## 1. Clarke Transform (3-Phase → αβ Stationary)

**Full Equations (Amplitude-Invariant)**:
- $I_\alpha = \frac{2}{3}(I_A - \frac{1}{2} I_B - \frac{1}{2} I_C)$
- $I_\beta = \frac{2}{3} \cdot \frac{\sqrt{3}}{2}(I_B - I_C) = \frac{\sqrt{3}}{3}(I_B - I_C)$

**Production Simplification**: In a balanced 3-phase system ($I_A + I_B + I_C = 0$), you only need 2 ADC readings. Substituting $I_C = -(I_A + I_B)$:

```c
#define FOC_INV_SQRT3 (0.57735027f)

/**
 * @brief Clarke transform using only 2 measured phase currents.
 *        Uses Kirchhoff's Current Law: Ic = -(Ia + Ib)
 *
 *        Production note: The 2/3 scaling factor is part of the
 *        amplitude-invariant convention. Some codebases absorb this
 *        constant into the PI gains to save 2 multiplies per ISR.
 *        If you do that, document it and ensure SVPWM input scaling matches.
 */
__attribute__((always_inline)) static inline void clarke_2ph(
                               const float32_t ia, const float32_t ib,
                               float32_t * const i_alpha, float32_t * const i_beta) {
    *i_alpha = ia;
    *i_beta  = (ia + 2.0f * ib) * FOC_INV_SQRT3;
}
```

**Why this is simpler**: With the 2-phase KCL form, the $\frac{2}{3}$ factor cancels partially, giving $I_\alpha = I_A$ and $I_\beta = \frac{1}{\sqrt{3}}(I_A + 2 I_B)$. This is the standard production form used in most FOC libraries. The full 3-phase form is only needed when you want to cross-check all three ADC readings for fault detection.

## 2. Park Transform (αβ Stationary → dq Rotating)

- $I_d =  I_\alpha \cos(\theta) + I_\beta \sin(\theta)$
- $I_q = -I_\alpha \sin(\theta) + I_\beta \cos(\theta)$

## 3. Inverse Park Transform (dq → αβ)

- $V_\alpha = V_d \cos(\theta) - V_q \sin(\theta)$
- $V_\beta = V_d \sin(\theta) + V_q \cos(\theta)$

Both Park and Inverse Park share the same sin/cos values. In a well-structured ISR, you compute sin/cos once (via CORDIC or FPU) and reuse for both directions.

## 4. STM32G4 CORDIC Initialization & Configuration

Before using the CORDIC coprocessor, it must be initialized once. The key registers:

```c
/**
 * @brief Initialize CORDIC for sin/cos computation in FOC ISR.
 *        Call once during system init, before any motor control starts.
 *
 *        Configuration:  Function = Cosine (outputs cos then sin)
 *                        Precision = 6 iterations (20-bit accuracy, ~6 cycles)
 *                        Input: 1 argument (angle only), 32-bit Q1.31
 *                        Output: 2 results (cos + sin),   32-bit Q1.31
 */
void cordic_init_sincos(void) {

    /* Enable CORDIC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;

    /* Configure CSR register:
     *   FUNC[3:0]    = 0b0000  → Cosine function (returns cos, then sin)
     *   PRECISION[3:0] = 6     → 6 iterations (good balance: ~20-bit accuracy)
     *   SCALE[2:0]   = 0       → No input scaling
     *   NARGS        = 0       → 1 argument (angle only, modulus defaults to 1.0)
     *   NRES         = 1       → 2 results (cos + sin)
     *   ARGSIZE      = 0       → 32-bit input
     *   RESSIZE      = 0       → 32-bit output
     *
     *   PRECISION trade-off: 4 iterations ≈ 13-bit, fast but noisy
     *                         6 iterations ≈ 20-bit, good for FOC
     *                         15 iterations = full 24-bit, rarely needed
     */
    CORDIC->CSR = (0u << CORDIC_CSR_FUNC_Pos)      /* Cosine function */
               | (6u << CORDIC_CSR_PRECISION_Pos)   /* 6 iterations */
               | (0u << CORDIC_CSR_SCALE_Pos)       /* No scaling */
               | (0u << CORDIC_CSR_NARGS_Pos)       /* 1 argument */
               | (1u << CORDIC_CSR_NRES_Pos)        /* 2 results (cos+sin) */
               | (0u << CORDIC_CSR_ARGSIZE_Pos)     /* 32-bit arg */
               | (0u << CORDIC_CSR_RESSIZE_Pos);    /* 32-bit result */
}
```

## 5. CORDIC Asynchronous Park Transform (Production Pattern)

The key production trick: write to CORDIC early in the ISR, do other work while it computes, then read results. The AHB bus stalls automatically if you read before computation finishes — no polling or interrupt needed.

```c
/* Q31 conversion constants — precomputed at compile time */
#define CORDIC_RAD_TO_Q31      (683565275.576f)   /* 2^31 / PI */
#define CORDIC_Q31_TO_FLOAT    (4.65661287e-10f)  /* 2^-31 */

/**
 * @brief  Asynchronous Park Transform using STM32G4 CORDIC.
 *         CORDIC must be pre-initialized via cordic_init_sincos().
 *
 *         Production usage pattern:
 *           1. cordic_write_angle(theta)     — fire-and-forget
 *           2. ... do Clarke, ADC reads, protection checks ...
 *           3. cordic_read_park(ia, ib, &id, &iq)  — collects results
 */

/** Step 1: Trigger CORDIC (non-blocking write) */
__attribute__((section(".ramfunc"))) __attribute__((always_inline)) static inline void cordic_write_angle(
                                                                    const float32_t theta_rad) {
    const int32_t q31_angle = (int32_t)(theta_rad * CORDIC_RAD_TO_Q31);
    CORDIC->WDATA = (uint32_t)q31_angle;
    /* CPU is now free — CORDIC computes in background */
}

/** Step 2: Read CORDIC results, complete Park transform, and OUTPUT sin/cos
 *          for reuse by Inverse Park (avoids recomputing trig). */
__attribute__((section(".ramfunc"))) __attribute__((always_inline)) static inline void cordic_read_park(
                                     const float32_t i_alpha, const float32_t i_beta,
                                     float32_t * const i_d, float32_t * const i_q,
                                     float32_t * const cos_out, float32_t * const sin_out) {
    /* RDATA read auto-stalls if CORDIC not finished yet */
    const float32_t cos_t = (float32_t)((int32_t)CORDIC->RDATA) * CORDIC_Q31_TO_FLOAT;
    const float32_t sin_t = (float32_t)((int32_t)CORDIC->RDATA) * CORDIC_Q31_TO_FLOAT;

    *i_d =  (i_alpha * cos_t) + (i_beta * sin_t);
    *i_q = -(i_alpha * sin_t) + (i_beta * cos_t);

    /* Return sin/cos for Inverse Park reuse */
    *cos_out = cos_t;
    *sin_out = sin_t;
}

/** Inverse Park using the SAME sin/cos from cordic_read_park */
__attribute__((section(".ramfunc"))) __attribute__((always_inline)) static inline void inv_park(
                            const float32_t v_d, const float32_t v_q,
                            const float32_t cos_t, const float32_t sin_t,
                            float32_t * const v_alpha, float32_t * const v_beta) {
    *v_alpha = (v_d * cos_t) - (v_q * sin_t);
    *v_beta  = (v_d * sin_t) + (v_q * cos_t);
}
```

### Typical ISR Pipeline (Production Flow)

```
ADC ISR Entry
  │
  ├─ 1. cordic_write_angle(theta_e)          ← CORDIC starts in background
  ├─ 2. Read ADC results (Ia, Ib, Vbus)      ← CPU works while CORDIC runs
  ├─ 3. clarke_2ph(Ia, Ib, &Ialpha, &Ibeta)
  ├─ 4. Run protection checks (OCP software layer, Vbus OVP)
  ├─ 5. cordic_read_park(Ialpha, Ibeta, &Id, &Iq, &cos_t, &sin_t)  ← Park + save trig
  ├─ 6. foc_current_update(Id, Iq → Vd, Vq)
  ├─ 7. inv_park(Vd, Vq, cos_t, sin_t → Valpha, Vbeta)
  ├─ 8. svpwm_generate(Valpha, Vbeta → DutyU, DutyV, DutyW)
  ├─ 9. dead_time_compensation()
  └─ 10. Write TIM1->CCR1/2/3
```

This pipeline typically completes in 3–8 µs on STM32G4 at 170 MHz, depending on compiler optimization, flash wait states, and whether critical functions are placed in CCMRAM (`.ramfunc`).

## ISR Timing Baseline Checklist

Capture and publish actual cycle budgets for each ISR phase on the target hardware and release build.

Illustrative example from one optimized STM32G4-class implementation:

- `cordic_write_angle` + ADC read: about 0.5-1.0 µs
- `clarke_2ph`: about 0.2 µs
- `cordic_read_park` + `foc_current_update` + `inv_park`: about 1.0-2.5 µs
- `svpwm_generate` + `dead_time_compensation`: about 1.0-2.5 µs
- TIM CCR update + post-processing: about 0.5 µs

**Illustrative total**: about 3.0-7.0 µs for 20kHz FOC in a well-optimized STM32G4 firmware. Treat this as a benchmark example, not a universal guarantee.

At minimum, publish:
- the measurement method (`DWT_CYCCNT`, scope GPIO toggle, or equivalent)
- build conditions (compiler flags, flash/RAM placement, debug vs release)
- hardware revision and clock tree
- worst-case operating points used during timing capture
