# STM32G4 FOC Hardware Specifics & PWM Synchronization (Reference)

## Overview
Do not rely purely on software for FOC timing limits and overcurrent shutdown. The STM32G4 has analog peripherals tailored precisely for driving half-bridges securely and efficiently.

The register choices and timing numbers below are typical STM32G4-oriented starting points. Always reconcile them with the exact timer mode, ADC sampling time, op-amp settling, gate-driver propagation delay, and board-level ringing on the real hardware.

## 1. ADC to PWM Synchronization (The Golden Rule)

If you sample phase current arbitrarily, you will sample inductive flyback, dead-time ringing, or diode reverse-recovery spikes. The sensor signal is only valid when the bottom FET is fully ON and ringing has subsided.

- **Synchronization Mechanic**: 
  The PWM timer (e.g., TIM1) runs in Center-Aligned Mode (Up-Down counting). You MUST link the `TIM1_TRGO` event to the ADC trigger.
  - Use a trigger source that maps to a known valid current-sampling instant. On STM32 advanced timers, a compare event such as `CCR4` is often preferable when you need a single, explicitly placed sampling point within the PWM period.
  - Do not assume the timer update event uniquely identifies the desired current valley in center-aligned mode; validate the exact trigger behavior against the timer mode and repetition-counter configuration.
  - **Latency Warning**: Account for op-amp settling, ADC acquisition time, propagation delay, and switching-node ringing. Place the trigger where the measured current is valid, not merely where the timer is geometrically centered.

## 2. Internal OPAMPs
STM32G4 includes ultra-fast internal OPAMPs (PGA mode) allowing you to directly route the physical shunt voltage to the MCU pin, eliminating external amplifier chips.
- Set PGA gain (x2, x4, x8, x16, x32, x64).
- Calibrate the OPAMP intrinsic offset (`OFR` register) during startup (while PWMs are disabled) and subtract this DC bias in software before feeding readings to Clarke transform. Re-check offset behavior across temperature and supply variation if accuracy is critical.

## 3. Zero-Cycle Hardware Trip (COMP -> TIM1_BRK)
Do NOT rely on the ADC ISR to shut down the drive. A low-resistance motor under a short circuit will ramp phase current past destruction levels in $< 5\mu s$. 
- **Rule**: Map the internal Analog Comparator (COMP) directly to the Shunt input.
- Set the DAC internally as the COMP negative reference when that architecture fits your current-limit strategy.
- Wire COMP output to `TIM1_BRK` (Break Input).
- **Result**: When current exceeds limits, the hardware break path reacts far faster than any software ISR path. Final shutdown latency still depends on comparator propagation, timer break handling, gate-driver behavior, and power-stage turn-off dynamics, so confirm it on the bench.

## 4. PWM Dead-Time & The Distortion Dilemma 

Gate driver shoot-through will vaporize the half-bridge. STM32 Advanced Timers provide hardware Dead-Time (`TIM_BDTR` register) to delay the turn-on of the complementary switch.

**The Physics Problem (Dead-Time Distortion)**: 
During the dead-time interval, neither FET is ON. Phase current freewheels through body diodes. The phase voltage is determined NOT by the PWM duty, but by the **polarity (sign) of the phase current**. This introduces a massive non-linear voltage error at zero-crossings, ruining low-speed FOC control and causing current THD.

```c
/**
 * @brief Compensates for Dead-time voltage errors.
 *        Apply this directly to phase voltage duties before updating timer registers.
 * 
 * @param dt_comp_value Precalculated duty fraction representing Dead-Time duration
 * @param i_phase Measured phase current 
 */
float32_t dead_time_compensation(float32_t raw_duty, float32_t i_phase, float32_t dt_comp_value) {
    /* If current is positive, body diode drops voltage to GND during DT. 
       If current is negative, body diode clamps it to VBUS during DT.  */
    
    if (i_phase > 0.1f) {
        return raw_duty + dt_comp_value; 
    } else if (i_phase < -0.1f) {
        return raw_duty - dt_comp_value;
    }
    
    /* Current near zero: Example no-compensation region to prevent chattering */
    return raw_duty; 
}
```
