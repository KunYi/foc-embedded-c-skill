# Protection & Diagnostics (Overload Estimation and Thermal Derating) (Reference)

## Overview

Industrial and automotive drives do not simply trip a fault the instant phase current exceeds the nominal rating. Motors and inverters have significant thermal inertia. A practical protection system allows short-term overload for acceleration or transient load steps, then derates or trips if the overload is sustained.

This document distinguishes between two different levels of protection logic:

- **`I^2t` overload estimator**: A practical heuristic that integrates current-related stress over time. It is simple, cheap, and very useful, but it is **not** a full physical temperature model.
- **Thermal state estimation**: A higher-fidelity model that attempts to estimate winding, inverter, or magnet temperature using calibrated thermal parameters and, ideally, measured temperatures.

For most embedded motor drives, start with `I^2t` plus measured temperature thresholds. Upgrade to a fuller thermal observer only when the product, duty cycle, or certification requirements justify the extra complexity.

---

## 1. What `I^2t` Is and Is Not

The `I^2t` algorithm is best understood as a **conservative overload budget**, not a literal thermal simulation.

- **What it captures well**: Sustained overload is more dangerous than short bursts, and copper heating grows roughly with current squared.
- **What it does not capture well**: True cooling dynamics, ambient temperature, airflow, inverter switching loss, iron loss, temperature-dependent winding resistance, and separate thermal masses for motor and power stage.

That means:

- `I^2t` is excellent for **protection and derating**.
- `I^2t` alone is not enough to claim you know the **actual motor temperature**.

### Practical Interpretation

Think of `I^2t` as a bucket of allowable overload energy:

- The bucket fills when current exceeds what the system can continuously sustain.
- The bucket empties when thermal stress falls below the continuous capability.
- When the bucket gets too full, the drive derates or trips.

This is a good product strategy because it is simple, deterministic, and robust even when direct temperature sensing is unavailable or noisy.

---

## 2. `I^2t` Overload Estimator

### Recommended Use

Use `I^2t` when you need:

- short-term peak torque without immediate nuisance trips
- deterministic derating behavior
- a fallback protection path even if temperature sensors are missing or faulty

Run it in a slow loop such as `1 kHz`, not in the high-frequency current ISR.

### C11 / MISRA-Friendly Reference Implementation

```c
#include <stdint.h>

#define FOC_ZERO        (0.0f)
#define I_NOMINAL       (10.0f)
#define I_PEAK          (30.0f)
#define I_NOMINAL_SQ    (I_NOMINAL * I_NOMINAL)
#define I2T_MAX_CAP     ((I_PEAK * I_PEAK - I_NOMINAL_SQ) * 3.0f)

typedef struct {
    float32_t accumulator;   /* Overload budget usage [A^2.s] */
    float32_t threshold;     /* Trip threshold [A^2.s] */
    float32_t warning_level; /* Derating threshold [A^2.s] */
    float32_t dt;            /* Update period [s] */
} i2t_model_t;

void i2t_init(i2t_model_t * const i2t) {
    i2t->accumulator = FOC_ZERO;
    i2t->threshold = I2T_MAX_CAP;
    i2t->warning_level = I2T_MAX_CAP * 0.8f;
    i2t->dt = 0.001f;
}

/**
 * @brief Update overload estimator from current magnitude.
 *
 * @param i_mag  Use total stator current magnitude when available:
 *               sqrt(Id^2 + Iq^2) or a calibrated RMS proxy.
 *               Using Iq alone is only acceptable if Id is known to remain small.
 *
 * @return 0 = normal, 1 = warning/derate, 2 = trip
 */
uint8_t i2t_update(i2t_model_t * const i2t, const float32_t i_mag) {
    const float32_t i_sq = i_mag * i_mag;

    /* Simple overload budget:
     * above nominal -> bucket fills
     * below nominal -> bucket empties */
    const float32_t delta = (i_sq - I_NOMINAL_SQ) * i2t->dt;
    i2t->accumulator += delta;

    if (i2t->accumulator < FOC_ZERO) {
        i2t->accumulator = FOC_ZERO;
    } else {
        /* Intentionally empty */
    }

    if (i2t->accumulator >= i2t->threshold) {
        return 2u;
    } else if (i2t->accumulator >= i2t->warning_level) {
        return 1u;
    } else {
        return 0u;
    }
}
```

### Important Limits of This Model

- It is **not** a physical winding-temperature estimator.
- The cooling term is only an engineering heuristic.
- The thresholds must be calibrated from real thermal tests, not chosen from current ratings alone.

---

## 3. Dynamic Torque Rollback (Derating)

Instead of hard-tripping as soon as the estimator approaches its limit, product-grade drives usually derate first.

```c
float32_t i2t_get_dynamic_limit(const i2t_model_t * const i2t) {
    if (i2t->accumulator < i2t->warning_level) {
        return I_PEAK;
    } else if (i2t->accumulator >= i2t->threshold) {
        return FOC_ZERO;
    } else {
        const float32_t range = i2t->threshold - i2t->warning_level;
        const float32_t excess = i2t->accumulator - i2t->warning_level;
        const float32_t ratio = excess / range;
        return I_PEAK - (ratio * (I_PEAK - I_NOMINAL));
    }
}
```

Use this output to limit the speed-loop or torque-loop current command rather than waiting for a final trip.

### Integration Guidance

- Clamp the outer-loop current request, not only the final PWM duty.
- Derating should be monotonic and predictable.
- Log the `I^2t` state for service diagnostics and field returns.

---

## 4. Measured Temperature Fusion

For product-level drives, `I^2t` should usually be combined with real temperature sensing:

- **Motor winding sensor**: NTC/PTC in the stator, if available
- **Power-stage sensor**: MOSFET, module, or heatsink NTC
- **PCB sensor**: local board temperature near the inverter

### Recommended Strategy

- Use `I^2t` as the **fast policy estimator for cumulative overload**
- Use measured temperature as the **ground truth safety limiter**
- Trip or derate on whichever condition is more conservative

### Example Fusion Policy

```c
typedef struct {
    float32_t motor_temp_c;
    float32_t inverter_temp_c;
    float32_t motor_warn_c;
    float32_t motor_trip_c;
    float32_t inverter_warn_c;
    float32_t inverter_trip_c;
} temp_limits_t;

uint8_t thermal_supervisor(const i2t_model_t * const i2t,
                           const temp_limits_t * const tlim) {
    const uint8_t i2t_state = (i2t->accumulator >= i2t->threshold) ? 2u :
                              (i2t->accumulator >= i2t->warning_level) ? 1u : 0u;

    if ((tlim->motor_temp_c >= tlim->motor_trip_c) ||
        (tlim->inverter_temp_c >= tlim->inverter_trip_c)) {
        return 2u;
    }

    if ((tlim->motor_temp_c >= tlim->motor_warn_c) ||
        (tlim->inverter_temp_c >= tlim->inverter_warn_c) ||
        (i2t_state == 1u)) {
        return 1u;
    }

    return i2t_state;
}
```

### Why This Is Better

- `I^2t` catches overload early even before a sensor warms up
- Temperature sensors catch real thermal stress that current alone does not model
- The combination is much closer to product-grade behavior than either alone

---

## 5. Calibration Workflow

This is the missing step in many control projects: protection logic is only as good as its calibration.

### Minimum Calibration Process

1. Define continuous current, peak current, and allowed peak duration targets.
2. Run controlled overload tests at multiple ambient temperatures.
3. Measure:
   - winding temperature rise
   - inverter/heatsink temperature rise
   - time to thermal steady state
   - time to critical threshold
4. Tune:
   - `I_NOMINAL`
   - `I_PEAK`
   - `I2T_MAX_CAP`
   - warning threshold
   - derating slope
5. Re-validate under:
   - low speed / stalled airflow
   - field weakening
   - regenerative braking
   - repeated acceleration cycles

### Product-Level Notes

- Calibrate motor and inverter protection separately if their safe operating limits differ materially.
- If winding resistance rises strongly with temperature, your `I^2t` thresholds may need temperature-dependent adjustment.
- If the cooling system changes by product variant, the calibration must change too.

---

## 6. Hardware Validation Checklist

To claim the overload/thermal protection is product-ready, verify these on real hardware:

- Apply repeated peak-torque pulses and confirm derating begins before measured temperatures exceed warning thresholds.
- Hold continuous overload and confirm trip occurs before winding or inverter temperature exceeds absolute safe limits.
- Compare `I^2t` accumulator shape against measured temperature rise over time. They do not need to match numerically, but they should correlate in trend and trigger order.
- Verify recovery behavior: after cooling, allowed torque should return smoothly without chatter.
- Verify sensor-fault fallback: if the temperature sensor opens/shorts, `I^2t` should still provide a conservative backup path.
- Verify logging: store cause of derating/trip (`I^2t`, motor temp, inverter temp, combined fault) for diagnostics.

---

## 7. Protection Hierarchy

A product-grade drive typically uses at least three protection layers:

1. **Hardware break path**: comparator to timer break for absolute short-circuit or catastrophic current events
2. **Fast software current supervision**: current-loop clamps and ADC plausibility checks
3. **Slow thermal supervision**: `I^2t`, measured temperatures, and derating policy

This layered design is what turns a mathematically good controller into a reliable product.
