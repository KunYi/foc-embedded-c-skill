# Space Vector PWM (SVPWM) Variants (Reference)

## Overview
This document transforms $V_\alpha$ and $V_\beta$ target voltages from the Inverse Clarke transform into physical Timer Duty cycles that drive the three-phase gates.

Treat the code here as a reference implementation shape, not the only acceptable realization. Final modulation choice must consider current-sensing windows, dead-time effects, minimum pulse width, EMC, switching loss, and hardware protection behavior.

## 1. Mathematical Bounds (Linear vs Hexagon)
Given a DC Bus voltage $V_{dc}$, the maximum phase voltage amplitude in the linear modulation region (inscribing the hexagon) is $V_{max} = \frac{V_{dc}}{\sqrt{3}}$. Exceeding this vector means you are over-modulating; the tips of your sine wave will flatten out.

```c
/**
 * @brief Transform V_alpha/V_beta into 7-Segment SVPWM Duty Cycles (Center Aligned)
 */
void svpwm_generate(float32_t v_alpha, float32_t v_beta, float32_t v_dc, 
                    float32_t *duty_u, float32_t *duty_v, float32_t *duty_w) {
                    
    /* 1. Calculate Phase Projections X, Y, Z */
    float32_t sqrt3_v_beta = 1.73205081f * v_beta;
    float32_t X = v_beta;
    float32_t Y = (sqrt3_v_beta + v_alpha) * 0.5f;
    float32_t Z = (sqrt3_v_beta - v_alpha) * 0.5f;

    /* 2. Determine Sector (1 to 6) based on Signs of X, Y, Z */
    uint8_t A = (X > 0.0f) ? 1 : 0;
    uint8_t B = (Y > 0.0f) ? 1 : 0;
    uint8_t C = (Z > 0.0f) ? 1 : 0;
    
    /* Example map translating binary flags to sectors 1-6 */
    uint8_t sector_map[] = {0, 2, 6, 1, 4, 3, 5, 0}; 
    uint8_t sector = sector_map[(A << 2) | (B << 1) | C];

    /* 3. Calculate Active Vector times T1, T2 */
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

    /* 4. Example over-modulation check and rescale */
    float32_t scale = v_dc / (t1 + t2);
    if (scale < 1.0f) {
        t1 *= scale;
        t2 *= scale;
    }

    /* 5. Determine Zero Vector time (T0 / 2) */
    float32_t t_zero_half = (v_dc - t1 - t2) * 0.5f;

    /* 6. Assign duties for Center-Aligned symmetric PWM (0.0 to 1.0) */
    float32_t ta = t_zero_half / v_dc;
    float32_t tb = (t_zero_half + t1) / v_dc;
    float32_t tc = (t_zero_half + t1 + t2) / v_dc;

    switch(sector) {
        case 1: *duty_u = tb; *duty_v = ta; *duty_w = tc; break;
        case 2: *duty_u = ta; *duty_v = tc; *duty_w = tb; break;
        case 3: *duty_u = ta; *duty_v = tb; *duty_w = tc; break;
        case 4: *duty_u = tc; *duty_v = tb; *duty_w = ta; break;
        case 5: *duty_u = tc; *duty_v = ta; *duty_w = tb; break;
        case 6: *duty_u = tb; *duty_v = tc; *duty_w = ta; break;
    }
}
```

## 2. 5-Segment Discontinuous PWM (DPWM)
At extremely high loads, switching heat ($\#FET_{transitions}$) is lethal. DPWM clamps one of the phase legs fully to $V_{dc}$ or GND for a 60-degree sector.
- **Constraints**: DPWM can reduce switching loss substantially, but it also increases distortion and interacts with current reconstruction. Avoid applying it blindly around operating regions where torque ripple, acoustic noise, or current-sensing windows become unacceptable. A modulation-index threshold can be useful, but the correct boundary is hardware- and application-dependent.
