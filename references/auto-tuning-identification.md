# Auto-Tuning & Parameter Identification (Reference)

## Overview
When motor parameters ($R_s, L_d, L_q, \Psi, P$) are unknown, standard mathematical FOC models fail instantly. A professional "Generic Drive" must execute a pre-flight identification routine (Auto-Tuning) to blindly deduce these physical traits before closing any operational loops.

The amplitudes, frequencies, and bandwidth examples in this document are commissioning starting points. They must be scaled to the motor's rated current, inductance, thermal limits, encoder availability, and inverter voltage margin.

## 1. Stator Resistance ($R_s$) Measurement
Resistance is purely resistive; you must eliminate inductance ($di/dt$) and Back-EMF ($\omega_e$) from the equation.

- **Procedure**:
  1. Hold the rotor completely at standstill ($\omega_e = 0$).
  2. Command a pure DC D-axis current with `I_q_cmd = 0A`. Start from a conservative current that is large enough to rise above ADC noise but comfortably below thermal and magnetic saturation limits.
  3. Wait for the PI loop to settle (so $di/dt = 0$).
  4. Measure the steady-state commanded voltage $V_d$.
- **Formula**: $R_s = V_d / I_{d\_meas}$
- **Safety**: Use only as much current as needed for a clean estimate, and account for copper heating because $R_s$ changes with temperature.
- **Production note**: Take multiple readings at different current levels to check for saturation effects. If $R_s$ varies significantly with current, the iron losses or winding proximity effects are non-negligible.

## 2. D/Q Inductance ($L_d, L_q$) Measurement
Inductance resists changes in current. High-frequency voltage injection removes resistive influence.

- **Procedure**:
  1. Rotor remains at standstill ($\omega_e = 0$).
  2. Inject a high-frequency voltage pulse or small-signal excitation into the D-axis ($V_d$), forcing $V_q = 0$. Choose the excitation frequency and amplitude high enough to reveal current ripple clearly, but low enough to stay inside hardware bandwidth and safe current limits.
  3. Sample the peak-to-peak current ripple $\Delta I_d$.
  4. Repeat the process for the Q-axis to find $L_q$.
- **Formula**: $L \approx \frac{V_{inject\_peak}}{\Delta I_{ripple}} \times \Delta t$
- **Rotor Saliency Detection**:
  - If $L_d \approx L_q$, the motor is Surface-Mounted PM (SPM).
  - If $L_q > L_d$ (often $1.5\sim3$ times), the motor is Interior PM (IPM), and you MUST enable MTPA (Maximum Torque Per Ampere) formulas.

## 3. Current Loop PI Zero-Pole Cancellation
Once you discover $R_s$ and $L_q, L_d$, you can derive a strong first-pass PI design algebraically instead of guessing gains manually. The resulting gains still need validation against digital delay, voltage saturation, sampling phase error, and current reconstruction limits.

```c
/**
 * @brief Auto-calculates a first-pass PI gain set for the current loop.
 *
 * Uses the continuous-domain zero-pole cancellation technique:
 *   Kp = L * bandwidth
 *   Ki = R * bandwidth
 *
 * This places the PI zero at the plant pole (R/L), giving a first-order
 * closed loop with the desired bandwidth.
 *
 * IMPORTANT: Digital delay limits.
 * The achievable bandwidth in a sampled system is bounded by:
 *   - 1 sample computational delay (current sample → duty update)
 *   - ZOH (Zero-Order Hold) on the PWM output
 *   - ADC + analog front-end group delay
 *
 * Rule of thumb: target bandwidth ≤ f_PWM / 10 as a safe starting point.
 * Aggressive designs may push to f_PWM / 5, but require careful phase
 * margin verification and possibly delay compensation (e.g., observer-based
 * current prediction or duty-cycle advance).
 *
 * Example: For f_PWM = 20 kHz:
 *   Conservative: BW ≤ 2 kHz  → bandwidth_rads ≈ 12566 rad/s
 *   Aggressive:   BW ≤ 4 kHz  → bandwidth_rads ≈ 25133 rad/s
 */
void calculate_current_pi_gains(float32_t r_s, float32_t l_d, float32_t l_q,
                                float32_t bandwidth_rads,
                                pi_controller_t *pi_id, pi_controller_t *pi_iq) {

    /* Zero-Pole Cancellation technique (continuous domain) */

    /* D-axis Controller */
    pi_id->kp = l_d * bandwidth_rads;
    pi_id->ki = r_s * bandwidth_rads;

    /* Q-axis Controller */
    pi_iq->kp = l_q * bandwidth_rads;
    pi_iq->ki = r_s * bandwidth_rads;

    /* Set output limits to ±Vdc/sqrt(3) or leave for the circle limiter */
    /* pi_id->out_max = v_bus_max * 0.577f; */
    /* pi_id->out_min = -pi_id->out_max; */
}
```

### Discrete-Time Considerations

For more advanced tuning that accounts for sampling delay explicitly:

- **Modified bandwidth formula**: With 1.5 Ts total delay (half-period sample + one-period compute), the effective phase margin erodes. A practical correction is:
  $BW_{safe} = \frac{2\pi f_{PWM}}{2\pi \cdot 1.5 \cdot T_s \cdot \text{PM\_factor}}$
  where PM_factor ≈ 5–10 depending on desired phase margin (60°–45°).

- **Tustin (bilinear) discretization**: Convert the continuous PI to discrete form properly. Do NOT simply multiply Ki by Ts and call it discrete — this introduces significant error if bandwidth approaches the sampling rate.

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

## 5. Speed Loop PI Tuning

Once the current loop is tuned and verified, the speed loop PI can be derived from the mechanical plant:

- **Plant model**: $\frac{\omega(s)}{I_q(s)} = \frac{K_t}{J \cdot s + B}$ where $K_t$ is the torque constant, $J$ is inertia, $B$ is viscous friction.
- **Bandwidth**: Target 1/10th of the current-loop bandwidth.
- **Rule of thumb**: $K_{p,spd} = J \times BW_{speed} / K_t$, $K_{i,spd} = B \times BW_{speed} / K_t$.
- In practice, $J$ and $B$ are often unknown. Start with conservative gains and increase until the speed response is satisfactory without oscillation.
