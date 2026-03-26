# Core Control & FOC Loops (Reference)

## Overview
This document covers the cascaded control loops for FOC: Position, Speed, and Current frames. The inner current loop (d-q axis) is the absolute priority. Execution latencies exceeding 20-30µs for this path will result in instability and poor THD.

The timing figures and code patterns below are reference-grade starting points. Final placement, acceleration choice, and bandwidth targets must be derived from the actual PWM rate, MCU clock, plant dynamics, and measurement delay.

## 1. Cross-Coupling Decoupling (PI Feed-forward)
The PM motor equations indicate that the d-axis and q-axis voltages are coupled entirely by the motor speed ($\omega_e$). To tune the PI controllers effectively as linear independent systems, you MUST decouple them.

**Mathematical Model:**
- $V_d = V_{d\_pi} - \omega_e \times L_q \times I_q$
- $V_q = V_{q\_pi} + \omega_e \times L_d \times I_d + \omega_e \times \Psi_f$

```c
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
void foc_current_update(foc_current_loop_t *foc, float32_t id_ref, float32_t iq_ref, 
                        float32_t id_meas, float32_t iq_meas, float32_t bus_voltage,
                        float32_t *vd_out, float32_t *vq_out) {
                        
    /* 1. Calculate base PI voltages */
    float32_t vd_pi = pi_update(&foc->pi_id, id_ref, id_meas);
    float32_t vq_pi = pi_update(&foc->pi_iq, iq_ref, iq_meas);
    
    /* 2. Calculate Feed-forward terms */
    float32_t vd_ff = -(foc->omega_e * foc->l_q * iq_meas);
    float32_t vq_ff =  (foc->omega_e * foc->l_d * id_meas) + (foc->omega_e * foc->flux_link);
    
    /* 3. Apply Decoupling */
    float32_t vd_target = vd_pi + vd_ff;
    float32_t vq_target = vq_pi + vq_ff;
    
    /* 4. Circle Limitation (Dynamic clamping based on Bus Voltage) */
    /* V_max = Bus / sqrt(3) */
    float32_t v_max = bus_voltage * 0.577350269f;
    float32_t v_sq  = (vd_target * vd_target) + (vq_target * vq_target);
    
    if (v_sq > (v_max * v_max)) {
        float32_t scale = v_max / sqrtf(v_sq); /* Heavy instruction, consider a faster path only if justified */
        vd_target *= scale;
        vq_target *= scale;
    }
    
    *vd_out = vd_target;
    *vq_out = vq_target;
}
```

## 2. Field Weakening (FW) & MTPA 
When driving an IPMSM (where $L_q > L_d$), setting $I_d = 0$ leaves torque geometry on the table.
- **Maximum Torque Per Ampere (MTPA)**: Inject negative $I_d$ proportional to $I_q$ to harvest reluctance torque. This is usually implemented as a Look-Up Table (LUT) populated during motor commissioning to avoid heavy non-linear roots calculations in the ISR.
- **Field Weakening**: Once $V_{cmd}$ hits the voltage hexagon limit (`v_max`), the speed controller must output negative $I_d$ to suppress $\Psi_f$, allowing higher RPMs at the cost of ohmic efficiency.

**Hardware Validation Criteria**:
To verify Field Weakening code is successful:
1. Rev the motor to maximum base RPM.
2. Probe DAC mapping to $I_d$. It should aggressively dive negative as you push the throttle past base speed.
## 3. Cascaded Outer Loops (Speed & Position)

A robust drive feeds the output of the Position loop into the Speed loop, and the Speed loop into the Current loop.

### A. The Bandwidth Rule (Frequency Separation)
Inner loops MUST execute exponentially faster than outer loops to remain mathematically decoupled. If you run the speed PI at the same frequency as the current PI without proper gains, the system will oscillate violently.
- **Rule of Thumb Ratio**: 10:1
- **Current Loop** ($I_q, I_d$): Commonly in the tens of kHz region, with bandwidth chosen well below switching frequency and adjusted for sensing delay, dead-time distortion, and plant inductance.
- **Speed Loop** ($\omega$): Commonly around one decade slower than the current loop, but tune relative to inertia, friction, load shocks, and sensor quality.
- **Position Loop** ($\theta$): Commonly another decade slower, unless the application deliberately pushes servo stiffness and has the sensing fidelity to support it.

### B. Position Loop (P Controller + Velocity Feed-Forward)
As a default, prefer a Proportional (P) position loop with velocity feed-forward because it is easy to stabilize and avoids many quantization and windup problems.
Integral or derivative action can still be valid in servo applications, but only when the sensing quality, anti-windup behavior, derivative filtering, and mechanical resonance risks are understood.

```c
/**
 * @brief Position controller yielding a target Speed [rad/s]
 */
float32_t foc_position_update(float32_t pos_target, float32_t pos_meas, 
                              float32_t velocity_feedforward, float32_t kp) {
    float32_t error = pos_target - pos_meas;
    
    /* Shortest-path phase wrapping for rotary systems [-PI, PI] */
    if (error > PI)  error -= TWO_PI;
    if (error < -PI) error += TWO_PI;
    
    float32_t speed_cmd = (error * kp) + velocity_feedforward;
    
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
float32_t foc_speed_update(pi_controller_t *pi_spd, float32_t spd_target, float32_t spd_meas, 
                           float32_t accel_feedforward, float32_t system_inertia_J) {
                           
    /* Base PI torque request */
    float32_t iq_pi = pi_update(pi_spd, spd_target, spd_meas);
    
    /* Inertia (Torque) Feed-forward: T = J * alpha */
    /* Scale Torque to Current: Iq_ff = T_ff / Kt */
    float32_t iq_ff = (accel_feedforward * system_inertia_J) / MOTOR_KT;
    
    float32_t iq_cmd = iq_pi + iq_ff;
    
    return clamp_f32(iq_cmd, -MAX_CURRENT, MAX_CURRENT);
}
```
