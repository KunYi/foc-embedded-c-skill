# Motor Protection & State Machine (Reference)

## Overview
FOC software must rigidly orchestrate the transition from a dead stop to a spinning rotor while maintaining hardware safety.

Treat the sequence and thresholds in this document as reference patterns, not universal constants. Current levels, dwell times, and trip thresholds must be scaled to the motor's thermal limits, inverter current capability, bus voltage, and mechanical risk.

## 1. FOC Global State Machine

```c
typedef enum {
    FOC_STATE_STANDBY = 0,     /* PWMs OFF, checking faults, ready to start */
    FOC_STATE_ALIGNMENT,       /* Forcing D-axis current to lock rotor */
    FOC_STATE_OPEN_LOOP,       /* V/f spinning rotor to build BEMF for observer */
    FOC_STATE_TRANSITION,      /* Observer converging, blending OL→CL */
    FOC_STATE_CLOSED_LOOP,     /* Full Sensorless/Sensor FOC active */
    FOC_STATE_FAULT,           /* Hardware trip or software protection engaged */
    FOC_STATE_FAULT_ASC,       /* Active Short Circuit fault state */
    FOC_STATE_FAULT_HIGHZ      /* High-Z coasting fault state */
} foc_state_t;
```

### Alignment (Open-Loop Lock)
For incremental encoders (QEP), the system powers up blind.
- You MUST force the state machine into `FOC_STATE_ALIGNMENT`.
- Apply a D-axis alignment current and set `I_q_ref = 0A`, then hold a fixed electrical angle such as $\theta = 0$. Start from the minimum current that reliably pulls the rotor into a known position.
- A common starting point is a modest fraction of rated current, then increase only as needed. Avoid using a hard-coded alignment current across all motors.
- Hold alignment only as long as needed to settle the rotor while staying within thermal limits. The correct dwell may range from a few milliseconds to hundreds of milliseconds depending on inertia, friction, and motor heating.
- Reset the TIM QEP counter to 0. Transition to `FOC_STATE_CLOSED_LOOP`.

## 2. Fault Protection Architectures

### A. Overcurrent (OCP)
- **Hardware Layer**: Handled exclusively by `COMP` to `TIM_BRK` (zero software latency).
- **Software Layer**: Read the status register in the background loop to confirm `TIM_BRK` fired. If fired, latch the software state to `FOC_STATE_FAULT` to disable high-level speed targets.

### B. DC-Bus Overvoltage (OVP) / Regenerative Braking
- Quickly decelerating a heavy inertia load forces the motor into a generator state ($I_q < 0$). This pumps flyback current directly into the DC Bus link capacitors.
- **Rule**: Define a bus overvoltage threshold with adequate margin below absolute capacitor, gate-driver, and inverter limits. If `V_bus` crosses that threshold, the software MUST engage the configured mitigation path.

**Brake Resistor Control** (if equipped):

```c
/**
 * @brief Hysteresis controller for external brake resistor / chopper.
 *        Activated when regenerative braking pumps the DC bus above safe limits.
 *
 *        The brake chopper is typically a single MOSFET + power resistor
 *        connected across the DC bus. PWM duty controls dissipation rate.
 */
typedef struct {
    float32_t vbus_brake_on;    /* Turn on threshold [V]  (e.g., 380V for 400V bus) */
    float32_t vbus_brake_off;   /* Turn off threshold [V] (e.g., 370V) */
    float32_t vbus_fault;       /* Hard fault threshold [V] (e.g., 420V → shutdown) */
    bool      brake_active;
} brake_chopper_t;

void brake_chopper_update(brake_chopper_t *brk, float32_t v_bus) {

    /* Hard fault: voltage exceeds absolute limit */
    if (v_bus > brk->vbus_fault) {
        /* Trigger emergency shutdown — see emergency-protection-halt.md */
        enter_emergency_highz();
        return;
    }

    /* Hysteresis control */
    if (v_bus > brk->vbus_brake_on) {
        brk->brake_active = true;
    } else if (v_bus < brk->vbus_brake_off) {
        brk->brake_active = false;
    }

    /* Drive brake chopper GPIO */
    if (brk->brake_active) {
        BRAKE_GPIO_PORT->BSRR = BRAKE_GPIO_PIN;   /* Brake ON: dissipate energy */
    } else {
        BRAKE_GPIO_PORT->BSRR = BRAKE_GPIO_PIN << 16u;  /* Brake OFF */
    }
}
```

**Alternative (no brake resistor)**: Reduce the deceleration rate by limiting the speed controller's negative $I_q$ command so regenerated power stays within the bus capacitor's absorption limit. This is slower but requires no extra hardware.

### C. Stall / Blocked Rotor Detection
- If the load completely jams, the current will saturate at maximum limits, but the rotor $\omega$ drops to zero.
- In sensorless mode, zero speed = no BEMF = the SMO observer diverges wildly.
- **Rule**: Detect stall from a sustained mismatch between torque-producing current and achieved speed or position. Use thresholds and time windows derived from the motor thermal time constant, expected load transients, and sensor resolution rather than relying on a universal current or RPM value.

## 3. The "Catch-Spin" / Flying Start
If the inverter reboots while the motor is already coasting at 3000 RPM (e.g., fan blade caught in wind), instantly firing a zero-angle PWM vector will cause a massive structural shock and overcurrent explosion.
- You must run the SMO Observer silently in the background (PWMs OFF, tracking ADC Back-EMF vectors) until the PLL locks onto the physical $\theta$ and $\omega$.
- Only enable the PWM outputs using the pre-locked $\theta$ angle.

## 4. Open-Loop → Closed-Loop Transition (Sensorless Startup)

This is one of the most critical and failure-prone sequences in sensorless FOC. A poorly designed transition causes current spikes, torque disturbances, or complete loss of control.

### A. Open-Loop (V/f) Ramp Phase

During `FOC_STATE_OPEN_LOOP`, the controller applies a rotating voltage vector without feedback:

```c
typedef struct {
    float32_t theta_ol;         /* Forced electrical angle [rad] */
    float32_t omega_ramp;       /* Current ramp speed [rad/s] */
    float32_t omega_target;     /* Target ramp speed (transition threshold) */
    float32_t v_amplitude;      /* Voltage amplitude (proportional to speed) */
    float32_t v_per_hz;         /* V/f ratio [V/(rad/s)] */
    float32_t ramp_rate;        /* Acceleration rate [rad/s²] */
    float32_t id_boost;         /* Initial D-axis current for rotor lock torque */
} open_loop_t;

/**
 * @brief Open-loop V/f ramp update. Run at current-loop rate.
 */
void open_loop_update(open_loop_t *ol, float32_t dt,
                      float32_t *theta_out, float32_t *vd_out, float32_t *vq_out) {

    /* Ramp speed gradually */
    if (ol->omega_ramp < ol->omega_target) {
        ol->omega_ramp += ol->ramp_rate * dt;
        if (ol->omega_ramp > ol->omega_target) {
            ol->omega_ramp = ol->omega_target;
        }
    }

    /* Integrate angle */
    ol->theta_ol += ol->omega_ramp * dt;
    if (ol->theta_ol > PI)  ol->theta_ol -= TWO_PI;
    if (ol->theta_ol < -PI) ol->theta_ol += TWO_PI;

    /* V/f voltage command
     * Vq drives the rotor forward, Vd provides initial pull-in torque.
     * Reduce Id boost as speed increases and the motor locks onto the field. */
    *vq_out = ol->omega_ramp * ol->v_per_hz;
    *vd_out = ol->id_boost * (1.0f - ol->omega_ramp / ol->omega_target);
    *theta_out = ol->theta_ol;
}
```

### B. Observer Convergence Criteria

Before switching from open-loop to closed-loop, the observer MUST demonstrate convergence. Check ALL of:

1. **BEMF amplitude exceeds threshold**: $|\hat{E}| = \sqrt{E_\alpha^2 + E_\beta^2} > E_{min}$. Below this, the observer is guessing, not tracking.
2. **PLL error is small**: The normalized PLL error (see `sensorless-observers.md`) should be below a tight threshold for a sustained period (e.g., 50 consecutive ISR cycles).
3. **Speed agreement**: The observer's estimated speed $\hat\omega$ should approximately match the open-loop forced speed $\omega_{OL}$ within a tolerance band (e.g., ±10%).
4. **Minimum speed reached**: The open-loop ramp must reach a speed where BEMF is large enough for reliable tracking.

```c
/**
 * @brief Check if sensorless observer is ready for closed-loop transition.
 */
bool is_observer_converged(const observer_pll_t *pll,
                           float32_t e_alpha, float32_t e_beta,
                           float32_t omega_ol,
                           uint32_t *converge_counter) {

    float32_t bemf_sq = (e_alpha * e_alpha) + (e_beta * e_beta);

    /* Condition 1: Sufficient BEMF */
    if (bemf_sq < BEMF_MIN_SQ) {
        *converge_counter = 0;
        return false;
    }

    /* Condition 2: Speed agreement between OL and observer */
    float32_t speed_err = fabsf(pll->omega_hat - omega_ol);
    if (speed_err > omega_ol * 0.15f) {  /* 15% tolerance */
        *converge_counter = 0;
        return false;
    }

    /* Condition 3: PLL tracking error is small */
    float32_t cos_th = cosf(pll->theta_hat);
    float32_t sin_th = sinf(pll->theta_hat);
    float32_t pll_err = (e_alpha * cos_th) + (e_beta * sin_th);
    pll_err /= sqrtf(bemf_sq);

    if (fabsf(pll_err) > 0.15f) {  /* Normalized error threshold */
        *converge_counter = 0;
        return false;
    }

    /* All conditions met — require sustained pass */
    (*converge_counter)++;
    return (*converge_counter >= CONVERGE_COUNT_THRESHOLD);  /* e.g., 50 cycles */
}
```

### C. Bumpless Transfer (The Handoff)

Simply switching from open-loop angle to observer angle causes a discontinuity. Production drives use a blending strategy:

```c
/**
 * @brief Bumpless OL→CL angle blending during transition.
 *        Gradually shifts from forced OL angle to observer angle.
 *
 * @param alpha    Blend factor: 0.0 = pure OL, 1.0 = pure CL
 * @param theta_ol Open-loop forced angle
 * @param theta_cl Observer estimated angle
 * @return         Blended angle for Park transform
 */
float32_t blend_angle(float32_t alpha, float32_t theta_ol, float32_t theta_cl) {

    /* Handle angle wrapping: compute the shortest angular difference */
    float32_t delta = theta_cl - theta_ol;
    if (delta >  PI) delta -= TWO_PI;
    if (delta < -PI) delta += TWO_PI;

    float32_t theta_blend = theta_ol + alpha * delta;

    /* Wrap result */
    if (theta_blend >  PI) theta_blend -= TWO_PI;
    if (theta_blend < -PI) theta_blend += TWO_PI;

    return theta_blend;
}
```

**Transition sequence**:
1. Enter `FOC_STATE_TRANSITION`.
2. Ramp blend factor `alpha` from 0.0 to 1.0 over a configurable period (e.g., 50–200 ms).
3. Simultaneously ramp open-loop voltage commands down and PI controller outputs up.
4. Pre-load PI integrators with the current voltage commands to prevent step discontinuity.
5. When `alpha = 1.0` and PI outputs are stable, switch to `FOC_STATE_CLOSED_LOOP`.

**Timeout**: If the observer fails to converge within a configurable timeout (e.g., 2 seconds), the state machine must fall back to `FOC_STATE_FAULT` — do NOT keep spinning in open loop indefinitely.
