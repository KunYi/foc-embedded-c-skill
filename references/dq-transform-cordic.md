# DQ Transform & CORDIC Integration (Reference)

## Overview
Field-Oriented Control involves continuous rotations of the reference frame using Clarke and Park transforms. The `sin(theta)` and `cos(theta)` operations mathematically dominate the cycle count.

The implementation examples below show a common STM32G4 optimization pattern. Use CORDIC when it improves deterministic timing or frees enough CPU budget to matter; plain FPU code may remain preferable during bring-up, low-rate control paths, or when simplicity outweighs the saved cycles.

## 1. Clarke / Park Equations (Amplitude-Invariant Form)
- **Clarke (3-phase to 2-phase stationary)**:
  - $I_\alpha = \frac{2}{3}(I_A - 0.5 \times I_B - 0.5 \times I_C)$
  - $I_\beta = \frac{2}{3}(\frac{\sqrt{3}}{2} I_B - \frac{\sqrt{3}}{2} I_C)$
- **Park (Stationary to Rotating frame)**:
  - $I_d = I_\alpha \cos(\theta) + I_\beta \sin(\theta)$
  - $I_q = -I_\alpha \sin(\theta) + I_\beta \cos(\theta)$

## 2. STM32G4 CORDIC Asynchronous Offloading

Software trigonometric paths are usually much slower than hardware CORDIC on STM32G4, but the exact cycle count depends strongly on the library, compiler, optimization settings, and whether you measure one function or an entire conversion path.
STM32G4 has a specific **CORDIC Coprocessor**.

**The Async Trick**: Avoid blocking on CORDIC when useful work can overlap its latency.
The standard approach is blocking (`write_cordic -> wait -> read_cordic`). 
A common high-performance STM32G4 strategy is asynchronous CORDIC:
1. First step in ISR: Trigger CORDIC computation by writing the raw angle `theta`.
2. Do other work: Fetch ADC readings, run protections, or calculate Clarke transforms while CORDIC crunches hardware math.
3. Read CORDIC results after enough useful work has elapsed for the result to be ready or nearly ready.

```c
/**
 * @brief Fast asynchronous Park Transform using STM32G4 CORDIC
 *        Requires CORDIC to be pre-configured in Cosine/Sine Mode (Function = 0)
 *        with 32-bit Q31 input/output formats.
 */
void cordic_async_park(float32_t i_alpha, float32_t i_beta, float32_t theta_rad, 
                       float32_t *i_d, float32_t *i_q) {
    
    /* 1. Convert float radian angle to Q31 format for CORDIC (-pi..pi -> -1..1) */
    int32_t q31_angle = (int32_t)(theta_rad * 683565275.57643f); /* theta / PI * 2^31 */
    
    /* 2. Write angle to CORDIC WDATA to TRIGGER hardware calculation asynchronously */
    CORDIC->WDATA = (uint32_t)q31_angle;
    
    /* ---- CPU IS NOW FREE FOR OVERLAPPED WORK ---- */
    /* (Execute Clarke transform or protection checks here if not already done.) */
    __NOP(); __NOP(); /* Example spacing */
    
    /* 3. Read back CORDIC RDATA (It automatically stalls CPU if not ready yet) */
    int32_t q31_cos = (int32_t)CORDIC->RDATA;
    int32_t q31_sin = (int32_t)CORDIC->RDATA; /* Read sequence yields Cos then Sin */
    
    /* 4. Convert back to float [-1.0f to 1.0f] */
    float32_t cos_t = (float32_t)q31_cos * 4.65661287e-10f; /* X * 2^-31 */
    float32_t sin_t = (float32_t)q31_sin * 4.65661287e-10f;

    /* 5. Complete Park Transform */
    *i_d = (i_alpha * cos_t) + (i_beta * sin_t);
    *i_q = (-i_alpha * sin_t) + (i_beta * cos_t);
}
```
