# STM32G4 FOC Hardware Specifics & PWM Synchronization (Reference)

## Overview
Do not rely purely on software for FOC timing limits and overcurrent shutdown. The STM32G4 has analog peripherals tailored precisely for driving half-bridges securely and efficiently.

## 1. ADC to PWM Synchronization (The Golden Rule)

If you sample phase current arbitrarily, you will sample inductive flyback, dead-time ringing, or diode reverse-recovery spikes. The sensor signal is only valid when the bottom FET is fully ON and ringing has subsided.

- **Synchronization Mechanic**: 
  The PWM timer (e.g., TIM1) runs in Center-Aligned Mode (Up-Down counting). You MUST link the `TIM1_TRGO` event to the ADC trigger.
  - Set `TIM_CR2_MMS = 010` (Update Event) or use `CCR4` to trigger TRGO.
  - The trigger MUST fire exactly at the **valley** of the PWM counter (when all bottom FETs are commonly ON in vector $V_0$).
  - **Latency Warning**: Consider the settling time of the OPAMP ($\approx 1\mu s$) and the ADC sample capability ($\approx 0.2\mu s$). Place `CCR4` a few counts before the absolute dead-bottom.

## 2. Internal OPAMPs
STM32G4 includes ultra-fast internal OPAMPs (PGA mode) allowing you to directly route the physical shunt voltage to the MCU pin, eliminating external amplifier chips.
- Set PGA gain (x2, x4, x8, x16, x32, x64).
- Calibrate the OPAMP intrinsic offset (`OFR` register) during startup (while PWMs are disabled) and subtract this DC bias in software before feeding reading to Clarke transform.

## 3. Zero-Cycle Hardware Trip (COMP -> TIM1_BRK)
Do NOT rely on the ADC ISR to shut down the drive. A low-resistance motor under a short circuit will ramp phase current past destruction levels in $< 5\mu s$. 
- **Rule**: Map the internal Analog Comparator (COMP) directly to the Shunt input.
- Set the DAC internally as the COMP negative reference (defining your Maximum Current Trip threshold).
- Wire COMP output to `TIM1_BRK` (Break Input).
- **Result**: When current exceeds limits, the hardware chops the PWM output to the gate driver in **nanoseconds**, bypassing the CPU entirely.

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
    
    /* Current near zero: No compensation to prevent chattering */
    return raw_duty; 
}
```
