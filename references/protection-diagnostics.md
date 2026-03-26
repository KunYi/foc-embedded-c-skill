# Protection & Diagnostics ($I^2t$ Overload Modeling) (Reference)

## Overview

Industrial and automotive drives do not simply trip a fault the instant phase current exceeds the nominal rating. Motors and inverters have significant **thermal mass**. An ideal protection system allows massive short-term overload (peak torque) for heavy acceleration, but dynamically pulls back the current limit or trips if the overcurrent is sustained too long.

We model this thermal energy accumulation using an **$I^2t$ (Ampere-squared-second)** algorithm.

---

## 1. The $I^2t$ Thermal Integrator (Leaky Bucket)

The $I^2t$ algorithm acts like a bucket of heat. 
- **Heat enters** at a rate proportional to $I_{measured}^2$.
- **Heat leaves** at a rate proportional to $I_{nominal}^2$ (the motor's continuous cooling capacity).
- **The bucket fills up** if $I_{measured} > I_{nominal}$.
- **The bucket empties** (cools down) if $I_{measured} < I_{nominal}$, eventually bottoming out at zero.
- If the bucket reaches its maximum capacity ($I^2t_{max}$ limit), the system must trip or aggressively derate the torque.

### C11 / MISRA Compliant Implementation

Because thermal time constants are long (seconds to minutes), this algorithm does NOT need to run in the high-frequency 20kHz ISR. It should run in a background task or a slow 1kHz timer tick.

```c
#include <stdint.h>
#include <stdbool.h>

/* --- Motor Thermal Specifications (Example Context) --- */
/* Nominal RMS current the motor can sustain indefinitely without overheating */
#define I_NOMINAL          (10.0f)
#define I_NOMINAL_SQ       (I_NOMINAL * I_NOMINAL)

/* Peak RMS current allowed for short bursts */
#define I_PEAK             (30.0f)

/* Maximum thermal energy accumulation before tripping.
 * Typical definition: How long can the motor sustain I_PEAK? 
 * I2t_MAX = (I_PEAK^2 - I_NOMINAL_SQ) * MAX_PEAK_TIME_SEC 
 * Example: 30A limit for 3 seconds over a 10A nominal rating */
#define I2T_MAX_CAPACITY   ((I_PEAK * I_PEAK - I_NOMINAL_SQ) * 3.0f)

/* Zero clamping boundary for the integrator */
#define FOC_ZERO           (0.0f)

typedef struct {
    float32_t accumulator;   /* Current accumulated thermal energy [A^2.s] */
    float32_t threshold;     /* Trip threshold limit [A^2.s] */
    float32_t warning_level; /* Warning threshold (e.g. 80% of limit) for soft-rollback */
    float32_t dt;            /* Execution period of this algorithm [s] (e.g., 0.001f for 1kHz) */
} i2t_model_t;

/**
 * @brief Initialize the I2t thermal model.
 */
void i2t_init(i2t_model_t * const i2t) {
    i2t->accumulator = FOC_ZERO;
    i2t->threshold = I2T_MAX_CAPACITY;
    i2t->warning_level = I2T_MAX_CAPACITY * 0.8f;
    i2t->dt = 0.001f; /* Assuming 1ms update rate */
}

/**
 * @brief  Update the fundamental thermal accumulator.
 *         Call this at a fixed low-frequency interval (e.g. 1kHz).
 * @param  i_q: Measured torque-producing current (or total magnitude sqrt(Id^2+Iq^2)).
 * @return status: 0 = Normal, 1 = Warning (Rollback requested), 2 = Tripped (Fault)
 */
uint8_t i2t_update(i2t_model_t * const i2t, const float32_t i_q) {
    
    /* Using Iq squared as the heat proxy. For FW systems, use (Id^2 + Iq^2). */
    const float32_t i_sq = (i_q * i_q);
    
    /* Calculate thermal power delta. 
     * Positive = heating up. Negative = cooling down. */
    const float32_t heat_delta = (i_sq - I_NOMINAL_SQ) * i2t->dt;
    
    /* Integrate */
    i2t->accumulator += heat_delta;
    
    /* Floor the accumulator at zero (ambient cooling equilibrium) */
    if (i2t->accumulator < FOC_ZERO) {
        i2t->accumulator = FOC_ZERO;
    } else {
        /* Intentionally empty */
    }
    
    /* Evaluate state */
    if (i2t->accumulator >= i2t->threshold) {
        return 2u; /* TRIP FAULT */
    } else if (i2t->accumulator >= i2t->warning_level) {
        return 1u; /* WARNING (DERATE TRIGGER) */
    } else {
        return 0u; /* NORMAL */
    }
}
```

---

## 2. Dynamic Torque Rollback (Derating)

Instead of brutally shutting down the motor when it gets close to overheating, a world-class drive performs a **Soft Rollback** (Derating). When the $I^2t$ accumulator crosses the warning threshold, the maximum allowed $I_q$ command in the speed loop is dynamically linearly restricted.

```c
/**
 * @brief  Calculate a dynamic torque limit based on the I2t thermal state.
 *         This output feeds into the MAX_CURRENT clamp of the Speed PI controller.
 *
 * @param i2t   The I2t model instance.
 * @return      The dynamically allowed maximum Iq current.
 */
float32_t i2t_get_dynamic_limit(const i2t_model_t * const i2t) {
    
    /* If perfectly cool, we allow the absolute peak burst limit. */
    if (i2t->accumulator < i2t->warning_level) {
        return I_PEAK;
    } 
    /* If we have exceeded the final threshold, we must be at 0A (fault state handles this)
     * but we clamp it math-wise here defensively. */
    else if (i2t->accumulator >= i2t->threshold) {
        return FOC_ZERO;
    } 
    /* We are between 80% (Warning) and 100% (Trip).
     * Linearly ramp down the allowed current from I_PEAK back down to I_NOMINAL. */
    else {
        const float32_t range = i2t->threshold - i2t->warning_level;
        const float32_t excess = i2t->accumulator - i2t->warning_level;
        
        /* ratio goes from 0.0 (at warning) to 1.0 (at trip limit) */
        const float32_t ratio = excess / range;
        
        /* Linearly squeeze the limit from I_PEAK down to I_NOMINAL */
        const float32_t dynamic_limit = I_PEAK - (ratio * (I_PEAK - I_NOMINAL));
        
        return dynamic_limit;
    }
}
```

### Integration with `control-foc-loops.md`

In your speed loop function, instead of using a hardcoded `#define MAX_CURRENT`:

```c
/* Example insertion in foc_speed_update() */
const float32_t active_i_max = i2t_get_dynamic_limit(&g_i2t_model);

/* Clamp the speed PI output */
const float32_t iq_cmd = pi_update(&pi_spd, spd_target, spd_meas) + iq_ff;
return clamp_f32(iq_cmd, -active_i_max, active_i_max);
```

### Protection Hierarchy

A world-class drive uses a triple-layered defense:
1.  **Hardware Break (Zero-cycle)**: Internal Analog `COMP` $\rightarrow$ `TIM1_BRK`. Triggers instantly beyond absolute max hardware limits (e.g., $50A$ short circuit).
2.  **Software Peak Limit (Current Loop)**: Clamps the reference in `foc_current_update` and verifies ADC readings against software safety limits. Triggers in ~50µs.
3.  **Thermal Integrator ($I^2t$)**: Evaluates time-over-current. Triggers in seconds to minutes. Protects stator varnish and limits average switching thermal stress.
