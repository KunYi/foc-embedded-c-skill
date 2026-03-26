# Sensorless Observers (SMO/PLL) (Reference)

## Overview
Hardware sensors break. Sensorless FOC estimates the rotor electrical angle $\theta_e$ by mathematically observing the Back-Electromotive Force (BEMF) on the non-driven axis of the rotor.

## 1. Sliding Mode Observer (SMO)

The SMO uses the motor equations to predict the current based on applied voltage. By feeding back the error between the measured current and predicted current through a non-linear (sliding) function, we force the mathematical observer to slide along the true BEMF trajectory.

- **Current Error Function**: $e_{i\alpha} = I_{\alpha\_{measured}} - I_{\alpha\_{hat}}$
- **Sliding Bang-Bang Compensation**: 
  $Z_\alpha = K_{smo} \times \text{sign}(e_{i\alpha})$
  *(To reduce chattering, use an approximated Sigmoid function instead of a hard sign.)*
- **BEMF Extraction**: Pass $Z_\alpha$ and $Z_\beta$ through a hardware/software Low-Pass Filter to extract smooth $\hat{E}_\alpha$ and $\hat{E}_\beta$.

## 2. Observer PLL Tracking (Angle Extraction)

Historically, systems used `atan2(E_beta, E_alpha)` to find the rotor angle. This is noisy.
Modern systems use an **Angle Tracking PLL** that locks onto the estimated BEMF.

```c
typedef struct {
    float32_t omega_hat;   /* Estimated electrical frequency */
    float32_t theta_hat;   /* Estimated electrical angle */
    float32_t kp;
    float32_t ki;
    float32_t dt;
} observer_pll_t;

/**
 * @brief PLL tracking loop for Sensorless Angle Extraction.
 *        Updates the estimated speed and angle based on SMO BEMF outputs.
 */
void update_pll(observer_pll_t *pll, float32_t e_alpha, float32_t e_beta) {
    /* 1. Calculate the orthogonal error function (delta angle approximation) 
          Error = -E_alpha * cos(theta) - E_beta * sin(theta)
          When error is 0, the PLL is perfectly aligned with the flux. */
          
    /* Utilize CORDIC lookup for these sin/cos calls for optimal speed */
    float32_t cos_th = cosf(pll->theta_hat);
    float32_t sin_th = sinf(pll->theta_hat);
    
    float32_t angle_error = (-e_alpha * cos_th) - (e_beta * sin_th);
    
    /* Normalize error by BEMF amplitude to decoupled PI gains from speed */
    float32_t bemf_amp_sq = (e_alpha * e_alpha) + (e_beta * e_beta);
    if (bemf_amp_sq > 0.1f) {
        /* Approximate FastInvSqrt could be used here */
        angle_error /= sqrtf(bemf_amp_sq);
    }
    
    /* 2. Standard PI Loop updating the estimated Speed */
    /* Note: simplified PI form */
    pll->omega_hat += pll->ki * angle_error * pll->dt; 
    float32_t omega_tracked = pll->omega_hat + (pll->kp * angle_error);
    
    /* 3. Integrate speed to get estimated angle */
    pll->theta_hat += omega_tracked * pll->dt;
    
    /* Wrap angle to [-pi, pi] */
    if (pll->theta_hat > PI) pll->theta_hat -= TWO_PI;
    if (pll->theta_hat < -PI) pll->theta_hat += TWO_PI;
}
```

## 3. High-Frequency Injection (HFI)
Note: SMO utterly fails at zero-speed because $\text{BEMF} \propto \omega$. For true zero-speed hold without physical sensors, you must rely on injecting $500Hz \sim 1kHz$ pulsating voltages on the d-axis and measuring the inductive envelope caused by magnetic saliency (only works on IPMSM).
