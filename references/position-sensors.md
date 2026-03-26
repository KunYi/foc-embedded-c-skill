# Position Sensors & Interpolation (Reference)

## Overview
If a motor has mechanical sensors, you gain extreme zero-speed torque but inherit digital latency, quantization noise, and mechanical misalignment constraints.

Use the methods here as reference patterns. The best interpolation, filtering, and alignment flow depends on encoder resolution, Hall placement error, SPI latency, rotor inertia, and required low-speed smoothness.

## 1. Quadrature Encoders (QEP/ABZ)

- **STM32 Hardware**: Route A/B signals to a Timer set in Encoder Mode (e.g., TIM2/TIM3). The counter automatically manages up/down direction based on phase shift.
- **Z-Index Alignment**: On startup, FOC doesn't know the absolute mapping between encoder `count=0` and rotor `angle=0`. Use an alignment method that matches the motor and mechanics, often including an open-loop current vector, then reset or calibrate the encoder reference accordingly.
- **Speed Calculation (M/T Method)**: Counting delta-pulses over a fixed ISR `dt` is noisy. Use a second timer to capture the Microseconds (T) between full pulse edges (M) to get high-resolution low-speed calculations.

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

## 3. Absolute SPI Encoders (Delay Compensation)
Sensors like AS5048 or optical BISS-C update absolute positions via high-speed SPI.
- **The Protocol Trap**: If SPI transfer plus sensor update latency is significant, the angle you receive is already old.
- **Compensation**: Extrapolate the angle forward before Park transformation: 
  $\theta_{foc} = \theta_{spi} + (\omega_{rotor} \times T_{comm\_delay})$
