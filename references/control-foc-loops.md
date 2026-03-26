# Core Control & FOC Loops (Reference)

## Overview
This document covers the cascaded control loops for FOC: Position, Speed, and Current frames. The inner current loop (d-q axis) is the absolute priority. Execution latencies exceeding 20-30µs for this path will result in instability and poor THD.

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
 *        This function MUST be placed in .ramfunc (CCMRAM) 
 *        to ensure deterministic execution (< 5us budget).
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
        float32_t scale = v_max / sqrtf(v_sq); /* Heavy instruction, consider FastInvSqrt */
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
3. Check the scope phase currents; they will no longer be perfectly sinusoidal, but you must prevent complete runaway if the inverter loses bus power during deep FW (leading to catastrophic overvoltage regeneration).
