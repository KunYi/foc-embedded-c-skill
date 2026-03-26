# Position Sensors & Interpolation (Reference)

## Overview
If a motor has mechanical sensors, you gain extreme zero-speed torque but inherit digital latency, quantization noise, and mechanical misalignment constraints.

Use the methods here as reference patterns. The best interpolation, filtering, and alignment flow depends on encoder resolution, Hall placement error, SPI latency, rotor inertia, and required low-speed smoothness.

## 1. Quadrature Encoders (QEP/ABZ)

- **STM32 Hardware**: Route A/B signals to a Timer set in Encoder Mode (e.g., TIM2/TIM3). The counter automatically manages up/down direction based on phase shift.
- **Z-Index Alignment**: On startup, FOC doesn't know the absolute mapping between encoder `count=0` and rotor `angle=0`. Use an alignment method that matches the motor and mechanics, often including an open-loop current vector, then reset or calibrate the encoder reference accordingly.

### Speed Estimation (M/T Method)

Simple speed = Δcount / Δt is noisy at low speeds because Δcount can be 0 or 1.

The **M/T method** uses input capture to timestamp individual encoder edges, giving high resolution at low speeds:

```c
typedef struct {
    uint32_t last_capture;      /* Timer capture value at last edge */
    float32_t omega_e;          /* Estimated electrical speed [rad/s] */
    float32_t rad_per_count;    /* Radians per encoder count (electrical) */
    float32_t timer_freq;       /* Capture timer frequency [Hz] */
    uint32_t  timeout_counts;   /* Max timer counts before declaring zero speed */
} encoder_speed_t;

/**
 * @brief Speed estimation from encoder edge input capture.
 *        Call from the Timer input capture ISR on each encoder edge.
 *
 *        At high speeds: many edges per ISR period → accurate.
 *        At low speeds: few edges but large timer counts → still accurate.
 *        At standstill: no edges → timeout detects zero speed.
 */
void encoder_speed_on_edge(encoder_speed_t *enc, uint32_t capture, int8_t direction) {
    uint32_t delta = capture - enc->last_capture;
    enc->last_capture = capture;

    if (delta > 0 && delta < enc->timeout_counts) {
        /* Speed = (radians_per_edge) / (time_per_edge) */
        enc->omega_e = (enc->rad_per_count * enc->timer_freq) / (float32_t)delta;
        enc->omega_e *= direction;  /* +1 or -1 from encoder direction */
    } else {
        enc->omega_e = 0.0f;  /* Timeout → standstill */
    }
}

/**
 * @brief Periodic check for zero-speed detection.
 *        Call from the current loop or a regular timer.
 *        If no encoder edge has arrived within the timeout, speed = 0.
 */
void encoder_speed_timeout_check(encoder_speed_t *enc, uint32_t current_timer) {
    uint32_t elapsed = current_timer - enc->last_capture;
    if (elapsed > enc->timeout_counts) {
        enc->omega_e = 0.0f;
    }
}
```

### Alternative: Delta-Count with Low-Pass Filter

For systems where input capture is unavailable, differentiate the count at a fixed rate and filter:

```c
/**
 * @brief Speed estimation via count differentiation + first-order LPF.
 *        Run at speed loop rate (e.g., 1kHz).
 */
void encoder_speed_diff(float32_t *omega_filtered, int32_t count_now, int32_t *count_prev,
                         float32_t rad_per_count, float32_t dt, float32_t lpf_alpha) {
    int32_t delta = count_now - *count_prev;
    *count_prev = count_now;

    float32_t omega_raw = (float32_t)delta * rad_per_count / dt;
    *omega_filtered += lpf_alpha * (omega_raw - *omega_filtered);
}
```

## 2. Hall Effect Sensors & Angle Interpolation

Hall sensors only provide 6 discrete 60-degree sectors per electrical revolution. If you feed a stair-stepping angle into the Park transform, your phase vectors will jump violently, causing massive audible noise and torque ripple.

**Constraint: Angle Interpolator**
You should usually use software to smoothly bridge the 60-degree gap between physical Hall transitions unless the application explicitly tolerates large torque ripple and acoustic noise.

```c
typedef struct {
    uint8_t curr_sector;      /**< 1 to 6 */
    float32_t base_angle_rad; /**< Absolute angle of the sector boundary */
    float32_t omega_e_radps;  /**< Filtered electrical speed */
    float32_t theta_interpolated;
} hall_interpolator_t;

/**
 * @brief Interpolates the angle continuously inside a 60-degree Hall State.
 *        Run this at a high enough rate that interpolation error stays below
 *        what the current loop can tolerate.
 */
void hall_angle_update(hall_interpolator_t *hall, float32_t dt) {
    /* Accumulate angle based on current speed */
    /* distance = speed * time */
    hall->theta_interpolated += hall->omega_e_radps * dt;

    /* Clamp the interpolation. It should not exceed the next expected sector boundary
       otherwise it can overshoot when the rotor slows or stalls. */

    float32_t max_angle_bound = hall->base_angle_rad + 1.04719755f; /* +60 degrees (PI/3) */

    /* Account for reverse rotation checks similarly depending on speed sign */
    if (hall->omega_e_radps > 0) {
        if (hall->theta_interpolated > max_angle_bound) {
             hall->theta_interpolated = max_angle_bound;
        }
    }
}
```

For Hall interpolation, keep the units consistent: the interpolated Hall angle is an electrical angle, so the integrated speed must also be electrical speed. If only mechanical speed is available, convert it using the motor pole-pair count before integrating.

### Hall Speed Estimation (Period Measurement)

```c
/**
 * @brief Speed estimation from Hall sector transition period.
 *        Uses TIM input capture to timestamp each Hall edge.
 *        Speed = 60° / measured_period.
 */
typedef struct {
    uint32_t last_hall_capture;
    float32_t omega_e;
    float32_t capture_timer_freq;
} hall_speed_t;

void hall_speed_on_transition(hall_speed_t *hs, uint32_t capture) {
    uint32_t period = capture - hs->last_hall_capture;
    hs->last_hall_capture = capture;

    if (period > 0) {
        /* 60° = PI/3 radians per Hall transition */
        hs->omega_e = (1.04719755f * hs->capture_timer_freq) / (float32_t)period;
    }
}
```

## 3. Absolute SPI Encoders (Delay Compensation)
Sensors like AS5048 or optical BISS-C update absolute positions via high-speed SPI.
- **The Protocol Trap**: If SPI transfer plus sensor update latency is significant, the angle you receive is already old.
- **Compensation**: Extrapolate the angle forward before Park transformation:
  $\theta_{foc} = \theta_{spi} + (\omega_{rotor} \times T_{comm\_delay})$
- **Practical note**: On STM32G4, SPI at 10 Mbps for a 16-bit transfer takes ~1.6 µs. The AS5048A internal processing adds ~10 µs. At 3000 RPM with 7 pole pairs, that's ~13° electrical delay — significant enough to degrade control at high speed.
