# Space Vector PWM (SVPWM) Variants (Reference)

## Overview
This document transforms $V_\alpha$ and $V_\beta$ target voltages from the Inverse Park transform into physical Timer Duty cycles that drive the three-phase gates.

Treat the code here as a reference implementation shape, not the only acceptable realization. Final modulation choice must consider current-sensing windows, dead-time effects, minimum pulse width, EMC, switching loss, and hardware protection behavior.

**Convention**: This implementation assumes the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling). The maximum inscribed circle voltage amplitude in the linear modulation region is $V_{max} = \frac{V_{dc}}{\sqrt{3}}$.

## 1. Mathematical Bounds (Linear vs Hexagon)
Given a DC Bus voltage $V_{dc}$, the voltage vector magnitude must satisfy $|\vec{V}| \le \frac{V_{dc}}{\sqrt{3}}$ to remain in the linear modulation zone (circle inscribed in the hexagon). Exceeding this boundary means over-modulation: the output voltage waveform clips against the hexagon edges, introducing harmonic distortion.

The full hexagon boundary allows up to $\frac{2}{3} V_{dc}$ peak fundamental on the phase, but only at the cost of low-order harmonics.

## 2. 7-Segment Center-Aligned SVPWM

### Compact Phase-Projection Method

This formulation determines the sector by computing projections that are algebraically equivalent to the phase voltages via Inverse Clarke. It saves multiplies compared to the textbook hexagonal-axis projection by reusing $\sqrt{3} V_\beta$ as a common subexpression. Widely used in production libraries (ST MC SDK, TI MotorWare, SimpleFOC).

**Mathematical equivalence**: $X = V_\beta$, $Y = \frac{\sqrt{3} V_\beta + V_\alpha}{2} = -V_c$, $Z = \frac{\sqrt{3} V_\beta - V_\alpha}{2} = V_b$. These three values partition the $\alpha\beta$-plane into 6 sectors identically to the standard hexagonal-axis formulation, merely expressed in a rotated and scaled basis. The sector map and $T_1, T_2$ assignments below are matched to this basis.

```c
/**
 * @brief Transform V_alpha/V_beta into 7-Segment SVPWM Duty Cycles (Center Aligned)
 *
 * Uses compact phase-projection sector determination.
 * Outputs duty cycles in [0.0, 1.0] range for TIM CCR = duty * ARR.
 *
 * Convention: Amplitude-invariant Clarke. Input V_alpha/V_beta are in volts.
 */
void svpwm_generate(float32_t v_alpha, float32_t v_beta, float32_t v_dc, 
                    float32_t *duty_u, float32_t *duty_v, float32_t *duty_w) {
                    
    /* 1. Calculate Phase Projections X, Y, Z
     *    X = Vβ
     *    Y = (√3·Vβ + Vα) / 2  ≡ -Vc  (negated phase-C voltage)
     *    Z = (√3·Vβ - Vα) / 2  ≡  Vb  (phase-B voltage)
     *
     *    This is algebraically equivalent to projecting onto the three
     *    hexagonal axes, but saves one multiply by sharing √3·Vβ.
     */
    float32_t sqrt3_v_beta = 1.73205081f * v_beta;
    float32_t X = v_beta;
    float32_t Y = (sqrt3_v_beta + v_alpha) * 0.5f;
    float32_t Z = (sqrt3_v_beta - v_alpha) * 0.5f;

    /* 2. Determine Sector (1 to 6) based on Signs of X, Y, Z
     *    The sign pattern maps uniquely to one of 6 sectors.
     *    Encoding: N = 4·sign(X) + 2·sign(Y) + sign(Z), where sign>0 => 1.
     */
    uint8_t A = (X > 0.0f) ? 1 : 0;
    uint8_t B = (Y > 0.0f) ? 1 : 0;
    uint8_t C = (Z > 0.0f) ? 1 : 0;
    
    static const uint8_t sector_map[8] = {0, 2, 6, 1, 4, 3, 5, 0}; 
    uint8_t sector = sector_map[(A << 2) | (B << 1) | C];

    /* 3. Calculate Active Vector times T1, T2
     *    These are un-normalized (in voltage units, proportional to Vdc).
     *    The switch-case selects the appropriate pair of projections
     *    for each sector, consistent with the X/Y/Z basis above.
     */
    float32_t t1, t2;
    switch(sector) {
        case 1: t1 = Z;  t2 = Y;  break;
        case 2: t1 = Y;  t2 = -X; break;
        case 3: t1 = -Z; t2 = X;  break;
        case 4: t1 = -X; t2 = Z;  break;
        case 5: t1 = X;  t2 = -Y; break;
        case 6: t1 = -Y; t2 = -Z; break;
        default: t1 = 0; t2 = 0; break;
    }

    /* 4. Normalize and Overmodulation clamp
     *    T1 and T2 are currently in voltage units. Normalize by Vdc
     *    to get time fractions relative to the switching period Ts.
     *    In the linear region: (t1 + t2) / Vdc <= 1.0.
     *    If exceeded, proportionally scale both to preserve angle
     *    while saturating magnitude at the hexagon boundary.
     */
    float32_t inv_vdc = 1.0f / v_dc;
    t1 *= inv_vdc;
    t2 *= inv_vdc;
    
    float32_t t_sum = t1 + t2;
    if (t_sum > 1.0f) {
        float32_t scale = 1.0f / t_sum;
        t1 *= scale;
        t2 *= scale;
    }

    /* 5. Determine Zero Vector time (T0 / 2) for symmetric 7-segment */
    float32_t t_zero_half = (1.0f - t1 - t2) * 0.5f;

    /* 6. Assign duties for Center-Aligned symmetric PWM (0.0 to 1.0)
     *    ta <= tb <= tc represent the three duty levels.
     *    The switch maps them to the correct phases per sector.
     */
    float32_t ta = t_zero_half;
    float32_t tb = t_zero_half + t1;
    float32_t tc = t_zero_half + t1 + t2;

    switch(sector) {
        case 1: *duty_u = tb; *duty_v = ta; *duty_w = tc; break;
        case 2: *duty_u = ta; *duty_v = tc; *duty_w = tb; break;
        case 3: *duty_u = ta; *duty_v = tb; *duty_w = tc; break;
        case 4: *duty_u = tc; *duty_v = tb; *duty_w = ta; break;
        case 5: *duty_u = tc; *duty_v = ta; *duty_w = tb; break;
        case 6: *duty_u = tb; *duty_v = tc; *duty_w = ta; break;
        default: *duty_u = 0.5f; *duty_v = 0.5f; *duty_w = 0.5f; break;
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
 *        Preferred when sector information is not needed downstream
 *        (e.g., 3-shunt sensing that doesn't depend on sector for reconstruction).
 */
void svpwm_minmax(float32_t v_alpha, float32_t v_beta, float32_t v_dc,
                  float32_t *duty_u, float32_t *duty_v, float32_t *duty_w) {

    /* Inverse Clarke (amplitude-invariant) to get phase voltages */
    float32_t v_u = v_alpha;
    float32_t v_v = (-0.5f * v_alpha) + (0.86602540f * v_beta);
    float32_t v_w = (-0.5f * v_alpha) - (0.86602540f * v_beta);

    /* Zero-sequence injection: center the waveform in [0, Vdc] */
    float32_t v_min = fminf(fminf(v_u, v_v), v_w);
    float32_t v_max = fmaxf(fmaxf(v_u, v_v), v_w);
    float32_t v_offset = -(v_max + v_min) * 0.5f;

    /* Normalize to [0, 1] duty range */
    float32_t inv_vdc = 1.0f / v_dc;
    *duty_u = (v_u + v_offset) * inv_vdc + 0.5f;
    *duty_v = (v_v + v_offset) * inv_vdc + 0.5f;
    *duty_w = (v_w + v_offset) * inv_vdc + 0.5f;
}
```

**Trade-off**: The min-max method is simpler and branchless, but does NOT produce sector information. If your firmware needs the current sector number for current reconstruction (1-shunt, 2-shunt) or for DPWM switching, use the sector-based method from Section 2 instead.
