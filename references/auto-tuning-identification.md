# Auto-Tuning & Parameter Identification (Reference)

## Overview
When motor parameters ($R_s, L_d, L_q, \Psi, P$) are unknown, standard mathematical FOC models fail instantly. A professional "Generic Drive" must execute a pre-flight identification routine (Auto-Tuning) to blindly deduce these physical traits before closing any operational loops.

## 1. Stator Resistance ($R_s$) Measurement
Resistance is purely resistive; you must eliminate inductance ($di/dt$) and Back-EMF ($\omega_e$) from the equation.

- **Procedure**: 
  1. Hold the rotor completely at standstill ($\omega_e = 0$).
  2. Command a pure DC D-axis current (e.g., $I_{d\_cmd} = 1.0A$, $I_{q\_cmd} = 0A$).
  3. Wait for the PI loop to settle (so $di/dt = 0$).
  4. Measure the steady-state commanded voltage $V_d$.
- **Formula**: $R_s = V_d / I_{d\_meas}$
- **Safety**: Do not exceed 10-20% of rated nominal current, otherwise you risk thermal damage to the stationary coils.

## 2. D/Q Inductance ($L_d, L_q$) Measurement
Inductance resists changes in current. High-frequency voltage injection removes resistive influence.

- **Procedure**:
  1. Rotor remains at standstill ($\omega_e = 0$).
  2. Inject a high-frequency voltage pulse (e.g., $10\text{kHz}$ Square Wave or Sine Wave) into the D-axis ($V_d$), forcing $V_q = 0$.
  3. Sample the peak-to-peak current ripple $\Delta I_d$.
  4. Repeat the process for the Q-axis to find $L_q$.
- **Formula**: $L \approx \frac{V_{inject\_peak}}{\Delta I_{ripple}} \times \Delta t$
- **Rotor Saliency Detection**: 
  - If $L_d \approx L_q$, the motor is Surface-Mounted PM (SPM).
  - If $L_q > L_d$ (often $1.5\sim3$ times), the motor is Interior PM (IPM), and you MUST enable MTPA (Maximum Torque Per Ampere) formulas.

## 3. Current Loop PI Zero-Pole Cancellation
Once you blindly discover $R_s$ and $L_q, L_d$, you no longer need to "tune" the Current Loop PI controller manually. You can algebraically assign perfect gains to achieve a target Bandwidth (e.g., $1000\text{Hz}$, $6283\text{ rad/s}$).

```c
/**
 * @brief Auto-calculates mathematically perfect PI gains for Current Loop.
 */
void calculate_current_pi_gains(float32_t r_s, float32_t l_d, float32_t l_q, 
                                float32_t bandwidth_rads, 
                                pi_controller_t *pi_id, pi_controller_t *pi_iq) {
    
    /* Zero-Pole Cancellation technique */
    /* Bandwidth usually set to 1/10th of PWM switching frequency */
    
    /* D-axis Controller */
    pi_id->kp = l_d * bandwidth_rads;
    pi_id->ki = r_s * bandwidth_rads;
    
    /* Q-axis Controller */
    pi_iq->kp = l_q * bandwidth_rads;
    pi_iq->ki = r_s * bandwidth_rads;
}
```

## 4. BEMF Flux Linkage ($\Psi$ / $K_e$) & Pole Pairs
These require motion limit testing and cannot be measured at standstill via simple injection.

- **Open-Loop Start**: To find them, force the system to spin using an Open-Loop `V/f` (Volts per Hertz) profile. Output a rotating voltage vector slowly ramping its frequency and amplitude until the rotor spins at a steady RPM without stalling.
- **Pole Pairs (P)**: 
  - If a mechanical sensor (Encoder/Hall) is equipped, spin $1$ mechanical revolution and count the number of electrical cycles ($\theta_e$ phase-wraps) executed on the stator variables. 
  - Formula: $P = \frac{\#\text{ Electrical Cycles}}{1\text{ Mechanical Rev}}$
- **Flux Linkage ($\Psi$)**:
  - Let the Sliding Mode Observer (SMO) estimate the Back-EMF components $E_\alpha, E_\beta$ during the Open-Loop spin.
  - Calculate Amplitude: $E_{mag} = \sqrt{E_\alpha^2 + E_\beta^2}$.
  - Formula: $\Psi = \frac{E_{mag}}{\omega_e}$. This flux linkage is required for Field Weakening and Speed Feed-forward loop accuracy.
