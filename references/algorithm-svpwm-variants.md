# Space Vector PWM (SVPWM) Variants (Reference)

## Overview
This document transforms $V_\alpha$ and $V_\beta$ target voltages from the Inverse Park transform into physical Timer Duty cycles that drive the three-phase gates.

Treat the code here as a reference implementation shape, not the only acceptable realization. Final modulation choice must consider current-sensing windows, dead-time effects, minimum pulse width, EMC, switching loss, and hardware protection behavior.

**Convention**: This implementation assumes the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling). In that convention, the commanded stationary-frame voltage vector magnitude satisfies $|\vec{V}_{\alpha\beta}| \le \frac{V_{dc}}{\sqrt{3}}$ in the linear modulation region.

## 1. Mathematical Bounds (Linear vs Hexagon)
Given a DC Bus voltage $V_{dc}$, the voltage vector magnitude must satisfy $|\vec{V}| \le \frac{V_{dc}}{\sqrt{3}}$ to remain in the linear modulation zone (circle inscribed in the hexagon). Exceeding this boundary means over-modulation: the output voltage waveform clips against the hexagon edges, introducing harmonic distortion.

The full hexagon boundary corresponds to a maximum stationary-frame reference vector magnitude of $\frac{2}{3}V_{dc}$. Driving beyond the inscribed circle toward this boundary enters overmodulation and trades waveform purity for more output voltage. Be explicit about which quantity you mean when discussing limits: phase fundamental, line-line fundamental, and $\alpha\beta$ vector magnitude are not interchangeable.

## 2. 7-Segment Center-Aligned SVPWM

### Compact Phase-Projection Method

This formulation determines the sector by computing projections that are algebraically equivalent to the phase voltages via Inverse Clarke. It saves multiplies compared to the textbook hexagonal-axis projection by reusing $\sqrt{3} V_\beta$ as a common subexpression. Widely used in production libraries (ST MC SDK, TI MotorWare, SimpleFOC).

**Mathematical equivalence**: $X = V_\beta$, $Y = \frac{\sqrt{3} V_\beta + V_\alpha}{2} = -V_c$, $Z = \frac{\sqrt{3} V_\beta - V_\alpha}{2} = V_b$. These three values partition the $\alpha\beta$-plane into 6 sectors identically to the standard hexagonal-axis formulation, merely expressed in a rotated and scaled basis. The sector map and $T_1, T_2$ assignments below are matched to this basis.

```c
#include <stdint.h>

#define FOC_SQRT3 (1.73205081f)
#define FOC_HALF  (0.5f)
#define FOC_ONE   (1.0f)
#define FOC_ZERO  (0.0f)

/**
 * @brief Transform V_alpha/V_beta into 7-Segment SVPWM Duty Cycles (Center Aligned)
 *
 * Uses compact phase-projection sector determination.
 * Outputs duty cycles in [0.0, 1.0] range for TIM CCR = duty * ARR.
 *
 * Convention: Amplitude-invariant Clarke. Input V_alpha/V_beta are in volts.
 */
__attribute__((always_inline)) static inline void svpwm_generate(
                    const float32_t v_alpha, const float32_t v_beta, const float32_t v_dc,
                    float32_t * const duty_u, float32_t * const duty_v, float32_t * const duty_w) {

    /* 1. Calculate Phase Projections X, Y, Z */
    const float32_t sqrt3_v_beta = FOC_SQRT3 * v_beta;
    const float32_t X = v_beta;
    const float32_t Y = (sqrt3_v_beta + v_alpha) * FOC_HALF;
    const float32_t Z = (sqrt3_v_beta - v_alpha) * FOC_HALF;

    /* 2. Determine Sector (1 to 6) based on Signs of X, Y, Z */
    const uint8_t a = (X > FOC_ZERO) ? 1u : 0u;
    const uint8_t b = (Y > FOC_ZERO) ? 1u : 0u;
    const uint8_t c = (Z > FOC_ZERO) ? 1u : 0u;

    static const uint8_t sector_map[8] = {0u, 2u, 6u, 1u, 4u, 3u, 5u, 0u};
    const uint8_t sector = sector_map[(a << 2u) | (b << 1u) | c];

    /* 3. Calculate Active Vector times T1, T2 */
    /* Normalize V_alpha and V_beta to the DC bus voltage */
    const float32_t inv_vdc = FOC_ONE / v_dc;
    const float32_t t_alpha = v_alpha * inv_vdc;
    const float32_t t_beta_sqrt3 = v_beta * inv_vdc * FOC_SQRT3;

    /* Calculate unscaled duty cycles for the active vectors */
    float32_t t1 = FOC_ZERO;
    float32_t t2 = FOC_ZERO;

    /* Sector 1: v_alpha > 0, v_beta > 0, v_beta < v_alpha * sqrt(3) */
    switch (sector) {
        case 1u:
            t1 = t_alpha - t_beta_sqrt3;
            t2 = t_beta_sqrt3 * 2.0f;
            break;
        case 2u:
            t1 = t_alpha + t_beta_sqrt3;
            t2 = t_beta_sqrt3 * 2.0f;
            break;
        case 3u:
            t1 = -(t_alpha - t_beta_sqrt3);
            t2 = -(t_alpha + t_beta_sqrt3);
            break;
        case 4u:
            t1 = -t_alpha - t_beta_sqrt3;
            t2 = -(t_beta_sqrt3 * 2.0f);
            break;
        case 5u:
            t1 = -t_alpha + t_beta_sqrt3;
            t2 = -(t_beta_sqrt3 * 2.0f);
            break;
        case 6u:
            t1 = t_alpha + t_beta_sqrt3;
            t2 = -(t_alpha - t_beta_sqrt3);
            break;
        default:
            __builtin_unreachable();
    }

    /* --- Overmodulation (OVM) Region 1 Handling ---
     * For a standard linear SVPWM, the maximum voltage vector must fit inside
     * the hexagon's inscribed circle: t1 + t2 <= 1.0.
     * If the user commands a voltage beyond this circle (e.g. into the corners
     * of the hexagon), we enter Overmodulation Region 1.
     * We stretch the active vectors to consume the entire PWM period,
     * forcing the zero-vector (t0) to identically zero. */
    const float32_t t_sum = t1 + t2;
    if (__builtin_expect(!!(t_sum > FOC_ONE), 0)) {
        const float32_t scale = FOC_ONE / t_sum;
        t1 *= scale;
        t2 *= scale;
    }

    /* 5. Determine Zero Vector time (T0 / 2) for symmetric 7-segment */
    const float32_t t_zero_half = (FOC_ONE - t1 - t2) * FOC_HALF;

    /* 6. Assign duties for Center-Aligned symmetric PWM (0.0 to 1.0) */
    const float32_t ta = t_zero_half;
    const float32_t tb = t_zero_half + t1;
    const float32_t tc = t_zero_half + t1 + t2;

    switch(sector) {
        case 1u: *duty_u = tb; *duty_v = ta; *duty_w = tc; break;
        case 2u: *duty_u = ta; *duty_v = tc; *duty_w = tb; break;
        case 3u: *duty_u = ta; *duty_v = tb; *duty_w = tc; break;
        case 4u: *duty_u = tc; *duty_v = tb; *duty_w = ta; break;
        case 5u: *duty_u = tc; *duty_v = ta; *duty_w = tb; break;
        case 6u: *duty_u = tb; *duty_v = tc; *duty_w = ta; break;
        default: __builtin_unreachable();
    }
}
```

### Minimum Pulse Width Constraint
When any duty approaches 0% or 100%, the resulting FET on-time may be shorter than the analog front-end needs to settle and sample current. This is critical for current reconstruction:

- **3-shunt**: If the bottom-FET on-time in any phase is too short, the ADC cannot capture a valid current sample in that phase. Software must either clamp the minimum duty, shift the sampling instant, or reconstruct from the remaining valid phases using the active sector.
- **1-shunt**: The minimum duration of each active vector state must exceed the op-amp settling plus ADC conversion time. If an active vector is too short, asymmetric PWM shifting or vector insertion is required (see `current-sensing-topology.md`).

The exact threshold depends on op-amp settling time, ADC acquisition time, and PCB ringing. Derive it from hardware measurements, not from a universal constant.

## 3. 5-Segment Discontinuous PWM (DPWM)
At extremely high modulation indices, switching loss becomes a dominant thermal concern. DPWM clamps one phase leg fully to either $V_{dc}$ or GND for each 60° sector, eliminating one-third of the switching transitions per cycle.

- **Implementation**: Replace the symmetric zero-vector split with an asymmetric allocation: set $T_{0,high} = 0$ (clamp-high DPWM) or $T_{0,low} = 0$ (clamp-low DPWM) on the clamped phase.
- **Constraints**: DPWM increases low-order harmonic distortion and interacts with current reconstruction (the clamped phase has a constant duty, which may eliminate one valid sensing window). Do not apply DPWM blindly around operating regions where torque ripple, acoustic noise, or current-sensing windows become unacceptable.
- **Transition Strategy**: A modulation-index threshold (e.g., switching from SVPWM to DPWM above $m > 0.9$) is a common approach, but the correct crossover point is hardware- and application-dependent. Beware of discontinuities in the voltage output at the SVPWM↔DPWM boundary.

## 4. Alternative: Mid-Point Clamped (Min-Max Injection)
A mathematically equivalent but branch-free alternative to the sector-based SVPWM above is the **min-max zero-sequence injection** method. It produces identical center-aligned waveforms without explicit sector computation:

```c
/**
 * @brief Simplified SVPWM via min-max zero-sequence injection.
 *        Produces the same output as 7-Segment SVPWM.
 *        Preferred when sector information is not needed downstream.
 */
__attribute__((always_inline)) static inline void svpwm_minmax(
                            const float32_t v_alpha, const float32_t v_beta, const float32_t v_dc,
                            float32_t * const duty_u, float32_t * const duty_v, float32_t * const duty_w) {

    /* Inverse Clarke (amplitude-invariant) */
    const float32_t v_u = v_alpha;
    const float32_t v_v = (-FOC_HALF * v_alpha) + (0.86602540f * v_beta);
    const float32_t v_w = (-FOC_HALF * v_alpha) - (0.86602540f * v_beta);

    /* Zero-sequence injection: center the waveform in [0, Vdc] */
    const float32_t v_min = fminf(fminf(v_u, v_v), v_w);
    const float32_t v_max = fmaxf(fmaxf(v_u, v_v), v_w);
    const float32_t v_offset = -(v_max + v_min) * FOC_HALF;

    /* Normalize to [0, 1] duty range */
    const float32_t inv_vdc = FOC_ONE / v_dc;
    *duty_u = (v_u + v_offset) * inv_vdc + FOC_HALF;
    *duty_v = (v_v + v_offset) * inv_vdc + FOC_HALF;
    *duty_w = (v_w + v_offset) * inv_vdc + FOC_HALF;
}
```

**Trade-off**: The min-max method is simpler and branchless, but does NOT produce sector information. If your firmware needs the current sector number for current reconstruction (1-shunt, 2-shunt) or for DPWM switching, use the sector-based method from Section 2 instead.

## 5. Software Dead-Time Compensation

Dead-time protects the inverter phase legs from shoot-through but causes a voltage error vector opposing the direction of phase current. A world-class drive injects a feed-forward compensation voltage before the Park/Inverse-Park transforms to cancel this distortion.

```c
/**
 * @brief Injects software dead-time compensation into target phase voltages.
 *        Runs BEFORE the Inverse Park or SVPWM block for each phase.
 *
 * @param v_target       Target Phase voltage (from Inverse Clarke/Park or direct)
 * @param i_meas         Measured Phase current
 * @param v_dead_comp    Pre-calculated dead-time error magnitude: (T_dead / T_pwm) * V_dc
 * @param i_threshold    Noise threshold to avoid compensation chatter at zero-crossing
 * @return               Compensated target voltage
 */
__attribute__((always_inline)) static inline float32_t svpwm_dead_time_compensate(
    const float32_t v_target,
    const float32_t i_meas,
    const float32_t v_dead_comp,
    const float32_t i_threshold) {

    float32_t v_comp = FOC_ZERO;

    if (i_meas > i_threshold) {
        v_comp = v_dead_comp;
    } else if (i_meas < -i_threshold) {
        v_comp = -v_dead_comp;
    } else {
        /* Linearly interpolate through the zero-crossing to prevent acoustic noise
           caused by the sign() flip triggering back and forth on AD noise. */
        v_comp = v_dead_comp * (i_meas / i_threshold);
    }

    return v_target + v_comp;
}
```

> [!TIP]
> Do not aggressively compensate dead-time during initial tuning. Verify the base current waveforms first. If sensing noise is high, the `sign()` flip will inject violent voltage transients at the zero-crossings, worsening THD instead of improving it.
