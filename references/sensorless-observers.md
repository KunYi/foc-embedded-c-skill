# Sensorless Observers (SMO/PLL) (Reference)

## Overview
Hardware sensors break. Sensorless FOC estimates the rotor electrical angle $\theta_e$ by mathematically observing the Back-Electromotive Force (BEMF) on the non-driven axis of the rotor.

The structures in this document are reference patterns. Observer form, gains, filtering, and convergence checks depend strongly on machine saliency, current-sensing quality, PWM frequency, startup method, and low-speed operating requirements.

**BEMF Sign Convention Used Here:**
For a PMSM with rotor flux aligned to the d-axis, the stator voltage equations in the stationary frame are:
- $V_\alpha = R_s I_\alpha + L \frac{dI_\alpha}{dt} + E_\alpha$
- $V_\beta  = R_s I_\beta  + L \frac{dI_\beta}{dt}  + E_\beta$

Where the BEMF terms are:
- $E_\alpha = -\Psi_f \omega_e \sin(\theta_e)$
- $E_\beta  =  \Psi_f \omega_e \cos(\theta_e)$

This means the BEMF vector **leads** the rotor flux (d-axis) by 90° electrical. The PLL error function below is derived from this convention.

## 1. Sliding Mode Observer (SMO)

The SMO uses the motor equations to predict the current based on applied voltage. By feeding back the error between the measured current and predicted current through a non-linear (sliding) function, we force the mathematical observer to slide along the true BEMF trajectory.

### Observer Equations (Discrete Form)

```c
typedef struct {
    float32_t i_alpha_hat;  /* Estimated alpha-axis current */
    float32_t i_beta_hat;   /* Estimated beta-axis current */
    float32_t e_alpha_lpf;  /* Low-pass filtered BEMF (alpha) */
    float32_t e_beta_lpf;   /* Low-pass filtered BEMF (beta) */
    float32_t r_s;          /* Stator resistance [ohm] */
    float32_t l_s;          /* Stator inductance [H] (use Ls = (Ld+Lq)/2 for IPM) */
    float32_t k_smo;        /* Sliding mode gain — must exceed max |BEMF| */
    float32_t lpf_alpha;    /* BEMF LPF coefficient (0..1, smaller = heavier filtering) */
    float32_t dt;           /* Sample period [s] */
} smo_observer_t;

/**
 * @brief  One-step SMO update. Execute at the current-loop rate.
 * @param  v_alpha, v_beta: Applied stator voltages (from inverse Park output)
 * @param  i_alpha, i_beta: Measured stator currents (from Clarke transform)
 */
void smo_update(smo_observer_t *smo,
                float32_t v_alpha, float32_t v_beta,
                float32_t i_alpha, float32_t i_beta) {

    /* 1. Current estimation error */
    float32_t e_ia = i_alpha - smo->i_alpha_hat;
    float32_t e_ib = i_beta  - smo->i_beta_hat;

    /* 2. Sliding-mode compensation (sigmoid approximation reduces chattering)
     *    A pure sign() function works but creates high-frequency noise.
     *    Sigmoid: z = K * e / (|e| + epsilon)  approximates sign() smoothly. */
    float32_t epsilon = 0.01f;  /* Boundary layer width — tune for noise vs. tracking */
    float32_t z_alpha = smo->k_smo * e_ia / (fabsf(e_ia) + epsilon);
    float32_t z_beta  = smo->k_smo * e_ib / (fabsf(e_ib) + epsilon);

    /* 3. Current observer update (Forward Euler)
     *    di_hat/dt = (1/L) * (V - R*i_hat - Z) */
    float32_t inv_l = 1.0f / smo->l_s;
    smo->i_alpha_hat += smo->dt * inv_l * (v_alpha - smo->r_s * smo->i_alpha_hat - z_alpha);
    smo->i_beta_hat  += smo->dt * inv_l * (v_beta  - smo->r_s * smo->i_beta_hat  - z_beta);

    /* 4. Extract BEMF from sliding-mode signal via low-pass filter
     *    The sliding compensation Z contains BEMF + switching noise.
     *    LPF recovers the fundamental BEMF. */
    smo->e_alpha_lpf += smo->lpf_alpha * (z_alpha - smo->e_alpha_lpf);
    smo->e_beta_lpf  += smo->lpf_alpha * (z_beta  - smo->e_beta_lpf);
}
```

- **Sliding Mode Gain ($K_{smo}$)**: Must be larger than the maximum expected BEMF magnitude ($\Psi_f \omega_{e,max}$). Too small: observer cannot track. Too large: more chattering noise to filter.
- **LPF Cut-off**: Trades tracking bandwidth against noise. The filter introduces phase lag in the estimated angle, which the PLL partially compensates. Start with a cut-off around 2–5× the maximum electrical frequency and tune on the bench.

## 2. Observer PLL Tracking (Angle Extraction)

Historically, systems used `atan2(E_beta, E_alpha)` to find the rotor angle. This is noisy and discontinuous at the ±π boundary. Modern systems use an **Angle Tracking PLL** that locks onto the estimated BEMF.

### PLL Error Function Derivation

Given the BEMF convention above ($E_\alpha = -\Psi \omega \sin\theta_e$, $E_\beta = \Psi \omega \cos\theta_e$), we construct an error signal that is zero when the estimated angle $\hat\theta$ equals the true angle $\theta_e$:

$$\epsilon = E_\alpha \cos(\hat\theta) + E_\beta \sin(\hat\theta)$$

Substituting the BEMF expressions:

$$\epsilon = -\Psi\omega\sin(\theta_e)\cos(\hat\theta) + \Psi\omega\cos(\theta_e)\sin(\hat\theta) = \Psi\omega\sin(\hat\theta - \theta_e)$$

For small tracking error, $\sin(\hat\theta - \theta_e) \approx \hat\theta - \theta_e$, giving a **linear error signal** that drives the PLL towards lock. The sign convention ensures **positive PI gains** produce stable tracking when $\omega > 0$.

```c
typedef struct {
    float32_t omega_hat;   /* Estimated electrical frequency [rad/s] */
    float32_t theta_hat;   /* Estimated electrical angle [rad] */
    float32_t kp;          /* PLL proportional gain */
    float32_t ki;          /* PLL integral gain */
    float32_t integrator;  /* PI integrator state */
    float32_t dt;          /* Sample period [s] */
} observer_pll_t;

/**
 * @brief PLL tracking loop for Sensorless Angle Extraction.
 *        Updates the estimated speed and angle based on SMO BEMF outputs.
 *
 *        Uses the standard error: ε = Eα·cos(θ̂) + Eβ·sin(θ̂)
 *        which equals Ψ·ω·sin(θ̂ - θe) and drives to zero at lock.
 */
void update_pll(observer_pll_t *pll, float32_t e_alpha, float32_t e_beta) {

    /* 1. Calculate sin/cos of estimated angle.
          Use CORDIC here when it meaningfully improves the platform budget. */
    float32_t cos_th = cosf(pll->theta_hat);
    float32_t sin_th = sinf(pll->theta_hat);

    /* 2. PLL error function (standard convention, positive PI gains stable)
     *    ε = Eα·cos(θ̂) + Eβ·sin(θ̂) = Ψ·ω·sin(θ̂ - θe) */
    float32_t angle_error = (e_alpha * cos_th) + (e_beta * sin_th);

    /* 3. Normalize error by BEMF amplitude to decouple PI gains from speed.
     *    This makes the PLL bandwidth approximately speed-independent. */
    float32_t bemf_amp_sq = (e_alpha * e_alpha) + (e_beta * e_beta);
    if (bemf_amp_sq > 0.01f) {
        angle_error /= sqrtf(bemf_amp_sq);
    }

    /* 4. PI controller for speed estimation */
    pll->integrator += pll->ki * angle_error * pll->dt;
    pll->omega_hat = pll->integrator + (pll->kp * angle_error);

    /* 5. Integrate speed to get estimated angle */
    pll->theta_hat += pll->omega_hat * pll->dt;

    /* 6. Wrap angle to [-π, π] */
    if (pll->theta_hat > PI)  pll->theta_hat -= TWO_PI;
    if (pll->theta_hat < -PI) pll->theta_hat += TWO_PI;
}
```

### PLL Gain Tuning Guidelines
- **Natural Frequency ($\omega_n$)**: Start with $\omega_n \approx 2\pi \times 20\text{Hz}$ to $2\pi \times 100\text{Hz}$ depending on speed range and noise.
- **Damping Ratio ($\zeta$)**: 0.7–1.0 (critically damped to slightly overdamped).
- **Gain Formulas**: $K_p = 2 \zeta \omega_n$, $K_i = \omega_n^2$.
- These assume the BEMF-normalized error ≈ angle error in radians near lock.

### Convergence Check
Before transitioning from open-loop to closed-loop, the PLL must demonstrate convergence:

```c
/**
 * @brief Check if the observer PLL has converged enough for closed-loop operation.
 *
 * @param cos_th, sin_th  Pre-computed trig from the most recent update_pll() call.
 *                        Avoids redundant cosf/sinf — reuse what's already computed.
 * @return true if observer is trustworthy, false if still converging.
 */
bool pll_is_converged(const observer_pll_t *pll, float32_t e_alpha, float32_t e_beta,
                      float32_t cos_th, float32_t sin_th) {

    /* 1. BEMF amplitude must be above minimum threshold.
     *    Below this, the observer is extrapolating, not tracking. */
    float32_t bemf_sq = (e_alpha * e_alpha) + (e_beta * e_beta);
    if (bemf_sq < BEMF_MIN_THRESHOLD_SQ) {
        return false;
    }

    /* 2. Normalized PLL error magnitude should be small.
     *    A locked PLL should have near-zero error. */
    float32_t err = (e_alpha * cos_th) + (e_beta * sin_th);
    err /= sqrtf(bemf_sq);

    if (fabsf(err) > PLL_LOCK_ERROR_THRESHOLD) {
        return false;  /* Error too large — not locked */
    }

    /* 3. Speed estimate should be positive and reasonable */
    if (pll->omega_hat < OMEGA_MIN_FOR_SENSORLESS) {
        return false;
    }

    return true;
}
```

## 3. High-Frequency Injection (HFI)

SMO degrades at very low speed because $\text{BEMF} \propto \omega$. For true zero-speed or near-zero-speed hold without physical sensors, an alternative estimation strategy is needed.

### When HFI Applies
- **IPM Motors Only**: HFI exploits magnetic saliency ($L_d \ne L_q$). It does NOT work on non-salient (SPM) machines.
- **Concept**: Inject a high-frequency voltage signal (typically 500Hz–2kHz) into the d-axis. The resulting current response differs between the d and q axes due to saliency. By demodulating this difference, the rotor position can be extracted at standstill.
- **Acoustic Impact**: The injected signal is often audible. Choose the frequency and amplitude within acoustic, thermal, and sensing constraints.
- **Handoff**: As the motor accelerates and BEMF grows, transition from HFI-based estimation to SMO/PLL-based estimation using a blending function weighted by speed.

Injection frequency, amplitude, demodulation filter design, and HFI↔SMO crossover speed are all application-dependent. If the motor is SPM (non-salient), HFI is not applicable — use open-loop startup followed by SMO transition instead.
