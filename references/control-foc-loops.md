# Core Control & FOC Loops (Reference)

## Overview
This document covers the cascaded control loops for FOC: Position, Speed, and Current frames. The inner current loop (d-q axis) is the absolute priority. Execution latencies exceeding 20-30µs for this path will result in instability and poor THD.

The timing figures and code patterns below are reference-grade starting points. Final placement, acceleration choice, and bandwidth targets must be derived from the actual PWM rate, MCU clock, plant dynamics, and measurement delay.

**Convention**: All voltage limits and scaling assume the **amplitude-invariant** Clarke transform ($\frac{2}{3}$ scaling). The linear modulation limit is $V_{max} = \frac{V_{dc}}{\sqrt{3}}$.

## 1. Cross-Coupling Decoupling (PI Feed-forward)
The PM motor equations indicate that the d-axis and q-axis voltages are coupled by the motor speed ($\omega_e$). Decoupling improves PI tuning by making the d and q axes behave as independent linear systems. At low speeds the coupling terms are negligible, but at moderate-to-high speeds they become significant and decoupling is strongly recommended.

**Mathematical Model:**
- $V_d = V_{d\_pi} - \omega_e \times L_q \times I_q$
- $V_q = V_{q\_pi} + \omega_e \times L_d \times I_d + \omega_e \times \Psi_f$

```c
#include <math.h>

#define FOC_INV_SQRT3 (0.577350269f)
#define FOC_ZERO      (0.0f)

typedef struct {
    pi_controller_t pi_id;
    pi_controller_t pi_iq;
    float32_t l_d;         /* d-axis inductance [H] */
    float32_t l_q;         /* q-axis inductance [H] */
    float32_t flux_link;   /* Flux linkage [V.s]    */
    float32_t omega_e;     /* Electrical speed [rad/s] */
} foc_current_loop_t;

/**
 * @brief High-frequency d-q current control with feed-forward decoupling.
 *        This function is often a candidate for .ramfunc (CCMRAM)
 *        or similar fast-memory placement when the platform budget is tight.
 */
__attribute__((section(".ramfunc")))
void foc_current_update(foc_current_loop_t * const foc,
                        const float32_t id_ref, const float32_t iq_ref,
                        const float32_t id_meas, const float32_t iq_meas,
                        const float32_t bus_voltage,
                        float32_t * const vd_out, float32_t * const vq_out) {

    /* 1. Calculate base PI voltages */
    const float32_t vd_pi = pi_update(&foc->pi_id, id_ref, id_meas);
    const float32_t vq_pi = pi_update(&foc->pi_iq, iq_ref, iq_meas);

    /* 2. Feed-forward decoupling terms
     *    Pre-multiply omega_e once to save a cycle */
    const float32_t we = foc->omega_e;
    const float32_t vd_ff = -(we * foc->l_q * iq_meas);
    const float32_t vq_ff =  (we * foc->l_d * id_meas) + (we * foc->flux_link);

    /* 3. Apply Decoupling */
    float32_t vd_target = vd_pi + vd_ff;
    float32_t vq_target = vq_pi + vq_ff;

    /* 4. Voltage Limitation (Dynamic clamping based on Bus Voltage)
     *    Standard linear limit: V_max_linear = Vdc / sqrt(3)
     *    OVM limit (Hexagon peak): V_max_ovm = 2 * Vdc / 3
     *    Set ENABLE_OVM macro externally to dictate the limit logic. */
#ifdef ENABLE_OVM
    /* Relax the PI clamp to the hexagon boundary (OVM allowed) */
    const float32_t v_max = bus_voltage * 0.66666667f; /* 2/3 */
#else
    /* Strictly linear modulation (Inscribed circle limit) */
    const float32_t v_max = bus_voltage * FOC_INV_SQRT3;
#endif

    const float32_t v_max_sq = v_max * v_max;
    const float32_t v_sq = (vd_target * vd_target) + (vq_target * vq_target);

    /* Use GCC unlikely constraint: saturation is the exception, not the rule */
    if (__builtin_expect(!!(v_sq > v_max_sq), 0)) {
        const float32_t scale = v_max / sqrtf(v_sq);
        vd_target *= scale;
        vq_target *= scale;

        /* Anti-windup: when voltage saturates, the PI integrators must
         * be frozen or corrected. Feed back the saturation to prevent windup. */
    } else {
        /* Intentionally empty */
    }

    *vd_out = vd_target;
    *vq_out = vq_target;
}
```

### PI Controller Reference Implementation

For completeness, here is a production-grade PI with anti-windup:

```c
typedef struct {
    float32_t kp;           /* Proportional gain */
    float32_t ki;           /* Integral gain */
    float32_t integrator;   /* Integral state */
    float32_t out_min;      /* Output lower clamp */
    float32_t out_max;      /* Output upper clamp */
} pi_controller_t;

/**
 * @brief PI controller with conditional-integration anti-windup.
 *        The integrator only accumulates when the output is not saturated.
 *        Suitable for current, speed, and FW loops.
 */
__attribute__((always_inline)) static inline float32_t pi_update(
                                   pi_controller_t * const pi,
                                   const float32_t ref, const float32_t meas) {
    const float32_t error = ref - meas;

    /* Proportional + Integral */
    float32_t output = (pi->kp * error) + pi->integrator;

    /* Clamp output */
    if (__builtin_expect(!!(output > pi->out_max), 0)) {
        output = pi->out_max;
    } else if (__builtin_expect(!!(output < pi->out_min), 0)) {
        output = pi->out_min;
    } else {
        /* Only integrate when not saturated (conditional integration) */
        pi->integrator += pi->ki * error;
    }

    return output;
}
```

## 2. Field Weakening (FW) & MTPA

### A. Maximum Torque Per Ampere (MTPA)

For IPM motors ($L_q > L_d$), setting $I_d = 0$ wastes reluctance torque. MTPA optimizes the Id/Iq split to maximize torque per unit current.

**MTPA Angle**: For a given current magnitude $I_s$:
$$\gamma_{MTPA} = \arcsin\left(\frac{-\Psi_f + \sqrt{\Psi_f^2 + 8(L_q - L_d)^2 I_s^2}}{4(L_q - L_d) I_s}\right)$$

**Production Implementation**: This nonlinear equation is too expensive for an ISR. Use a LUT generated during commissioning:

```c
/**
 * @brief MTPA lookup: given desired torque current magnitude,
 *        returns optimal Id reference for IPM reluctance harvest.
 *
 *        LUT is populated offline during motor commissioning by
 *        sweeping Is and computing the MTPA angle analytically.
 */
typedef struct {
    float32_t is_table[MTPA_LUT_SIZE];   /* Current magnitude breakpoints [A] */
    float32_t id_table[MTPA_LUT_SIZE];   /* Corresponding optimal Id [A] */
    uint16_t  size;
} mtpa_lut_t;

float32_t mtpa_lookup(const mtpa_lut_t * const lut, const float32_t iq_cmd) {
    const float32_t is_mag = fabsf(iq_cmd);  /* Simplified: assume Id_mtpa << Iq initially */

    /* Linear interpolation across LUT */
    for (uint16_t i = 0u; i < (lut->size - 1u); i++) {
        if (is_mag <= lut->is_table[i + 1u]) {
            const float32_t frac = (is_mag - lut->is_table[i])
                                 / (lut->is_table[i + 1u] - lut->is_table[i]);
            return lut->id_table[i] + (frac * (lut->id_table[i + 1u] - lut->id_table[i]));
        } else {
            /* MISRA: Intentionally empty to proceed with search */
        }
    }
    return lut->id_table[lut->size - 1u];  /* Saturate at max */
}
```

For SPM motors ($L_d \approx L_q$), MTPA reduces to $I_d = 0$ and MTPA logic can be bypassed entirely.

### B. Field Weakening (FW)

When the voltage command approaches the hexagon limit, the motor has reached its **base speed**. To go faster, inject negative $I_d$ to suppress flux, reducing back-EMF and allowing higher RPM at the cost of copper loss.

**Production FW Implementation** — Voltage-feedback FW regulator:

```c
typedef struct {
    pi_controller_t pi_fw;    /* FW PI: input = voltage margin, output = Id_fw */
    float32_t v_max_sq;       /* Pre-squared voltage limit (updated each cycle) */
    float32_t id_fw_min;      /* Maximum negative Id for FW (motor demagnetization limit) */
} field_weakening_t;

/**
 * @brief Voltage-feedback field weakening controller.
 *        Call AFTER foc_current_update() to check if voltage saturated.
 */
void field_weakening_update(field_weakening_t * const fw,
                            const float32_t vd, const float32_t vq, const float32_t bus_voltage,
                            const float32_t id_base, const float32_t iq_base,
                            float32_t * const id_ref_out, float32_t * const iq_ref_out) {

    /* 1. Compute voltage margin (how close to the limit) */
#ifdef ENABLE_OVM
    const float32_t v_max = bus_voltage * 0.66666667f;
#else
    const float32_t v_max = bus_voltage * FOC_INV_SQRT3;
#endif
    fw->v_max_sq = v_max * v_max;
    const float32_t v_cmd_sq = (vd * vd) + (vq * vq);

    /* 2. FW PI regulator: drives voltage to just below the limit. */
    const float32_t voltage_error = v_cmd_sq - (fw->v_max_sq * 0.9025f);
    float32_t id_fw = pi_update(&fw->pi_fw, FOC_ZERO, -voltage_error);

    /* 3. Clamp FW Id to prevent demagnetization */
    if (id_fw < fw->id_fw_min) {
        id_fw = fw->id_fw_min;
    } else {
        /* Intentionally empty */
    }

    if (id_fw > FOC_ZERO) {
        id_fw = FOC_ZERO;  /* FW only injects negative Id */
    } else {
        /* Intentionally empty */
    }

    /* 4. Apply current circle constraint: Id² + Iq² ≤ Imax² */
    const float32_t id_total = id_base + id_fw;
    const float32_t iq_limit = sqrtf(fmaxf(MAX_CURRENT_SQ - (id_total * id_total), FOC_ZERO));
    const float32_t iq_final = clamp_f32(iq_base, -iq_limit, iq_limit);

    *id_ref_out = id_total;
    *iq_ref_out = iq_final;
}
```

**Hardware Validation Criteria**:
To verify Field Weakening code is successful:
1. Rev the motor to maximum base RPM.
2. Probe DAC mapping to $I_d$. It should aggressively dive negative as you push the throttle past base speed.
3. $V_{cmd}$ magnitude should stabilize at ~95% of $V_{max}$ (squared threshold 0.9025) and stay there — the FW PI is regulating it.
4. If $I_d$ hits the demagnetization limit and speed still rises, the system will lose control — verify this boundary on the bench.

## 3. Cascaded Outer Loops (Speed & Position)

A robust drive feeds the output of the Position loop into the Speed loop, and the Speed loop into the Current loop.

### A. The Bandwidth Rule (Frequency Separation)
Inner loops must execute sufficiently faster than outer loops to remain approximately decoupled. Running cascaded loops at similar bandwidths without careful gain design will cause oscillation.
- **Common Starting Ratio**: 5:1 to 10:1. Tighter ratios (3:1) are achievable with proper digital delay compensation and verified phase margin.
- **Current Loop** ($I_q, I_d$): Commonly in the kHz range, with bandwidth chosen well below switching frequency and adjusted for sensing delay, dead-time distortion, and plant inductance.
- **Speed Loop** ($\omega$): Typically one decade slower than the current loop, but tune relative to inertia, friction, load transients, and sensor quality.
- **Position Loop** ($\theta$): Typically another factor slower, unless the application deliberately pushes servo stiffness and has the sensing fidelity to support it.

### Supervisory Target Conditioning and Bumpless Handover
In a product drive, the outer loops are usually fed by a supervisory source such as UART, CAN, PWM input, or a host trajectory generator rather than a fixed local constant.

Before a host-level target reaches the speed or position loop:
- apply range checks in engineering units
- apply ramp, acceleration, or jerk limits consistent with the mechanics
- reject stale or invalid commands before they reach the loop integrators
- define a deterministic handover rule when switching between torque, speed, and position modes

Hard rule: do not let a mode switch or host packet boundary instantaneously inject a hidden integrator mismatch into the outer loop. If the target owner changes, either:
- freeze and reinitialize the outgoing loop state, or
- preload the incoming loop state so the resulting torque/current stays continuous within documented limits

Acceptance criteria for supervisory handover:
- switching from speed mode to torque mode does not create an uncontrolled current step
- switching from position mode to speed mode does not leave the speed integrator wound up from the previous mode
- a lost host command causes the outer-loop target to follow the documented fallback policy rather than holding an unsafe stale request indefinitely

### B. Position Loop (P Controller + Velocity Feed-Forward)
As a default, prefer a Proportional (P) position loop with velocity feed-forward because it is easy to stabilize and avoids many quantization and windup problems.
Integral or derivative action can still be valid in servo applications, but only when the sensing quality, anti-windup behavior, derivative filtering, and mechanical resonance risks are understood.

```c
/**
 * @brief Position controller yielding a target Speed [rad/s]
 *
 *        Uses Proportional control with velocity feed-forward.
 *        Can handle shortest-path phase wrapping for single-turn rotary systems.
 *
 * @param pos_target          Desired position [rad]
 * @param pos_meas            Measured position [rad] (e.g., from encoder, absolute sensor)
 * @param velocity_feedforward Feed-forward velocity [rad/s] (optional, set to 0 for P-only)
 * @param kp                  Position proportional gain [rad/s per rad_error]
 * @return                    Target speed to feed to speed loop [rad/s]
 *
 * Strategy: Prefer P-control for position to avoid integrator windup.
 * If derivative action is needed, perform it with appropriate D-section filtering
 * to suppress encoder noise and quantization effects.
 */
__attribute__((always_inline)) static inline float32_t foc_position_update(
                                   const float32_t pos_target, const float32_t pos_meas,
                                   const float32_t velocity_feedforward, const float32_t kp) {

    float32_t error = pos_target - pos_meas;

    /* Shortest-path phase wrapping for single-turn rotary systems [-PI, PI].
     * For multi-turn or linear actuators, omit this block. */
    const float32_t pi = 3.14159265f;
    const float32_t two_pi = 6.28318531f;

    /* Map error into [-PI, PI] range to avoid taking the long way around */
    if (error > pi) {
        error -= two_pi;
    } else if (error < -pi) {
        error += two_pi;
    } else {
        /* Error already in valid range — intentionally empty */
    }

    /* Proportional term + velocity feed-forward */
    const float32_t speed_cmd = (error * kp) + velocity_feedforward;

    /* Clamp target speed to mechanical limits */
    return clamp_f32(speed_cmd, -MAX_OMEGA, MAX_OMEGA);
}
```

### C. Speed Loop (PI Controller + Inertia Feed-Forward)
The speed loop determines the $I_q$ torque request. During aggressive acceleration, waiting for the PI error to integrate to the required torque causes a massive following error (lag).
If you know the mechanical inertia ($J$) of the system, inject the torque required to accelerate the mass instantly.

```c
/**
 * @brief Speed controller yielding a target Torque (I_q) [A]
 */
float32_t foc_speed_update(pi_controller_t * const pi_spd,
                           const float32_t spd_target, const float32_t spd_meas,
                           const float32_t accel_feedforward, const float32_t system_inertia_j) {

    /* Base PI torque request */
    const float32_t iq_pi = pi_update(pi_spd, spd_target, spd_meas);

    /* Inertia (Torque) Feed-forward: T = J * alpha */
    /* Scale Torque to Current: Iq_ff = T_ff / Kt */
    const float32_t iq_ff = (accel_feedforward * system_inertia_j) * INV_MOTOR_KT;  /* precompute 1/Kt */

    const float32_t iq_cmd = iq_pi + iq_ff;

    return clamp_f32(iq_cmd, -MAX_CURRENT, MAX_CURRENT);
}
```

### Speed-Loop Validation Under Host Commands
When the speed loop is driven by a host command path instead of a local knob:

- log both the raw commanded speed and the conditioned internal speed target
- verify packet jitter or supervisory scheduling jitter does not appear as torque ripple at the motor
- step the host command and confirm the resulting `Iq_ref` respects current limits and acceleration policy
- validate timeout behavior explicitly: the speed loop should decay, hold, or fault exactly as the product definition requires
