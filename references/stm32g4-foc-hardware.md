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
#include <stdint.h>

/**
 * @brief Compensates for Dead-time voltage errors.
 *        Apply this directly to phase voltage duties before updating timer registers.
 * 
 * @param raw_duty      Uncorrected duty cycle [0.0 .. 1.0]
 * @param i_phase       Measured phase current [A]
 * @param dt_comp_value Precalculated duty fraction: dead_time_ns / (PWM_period_ns / 2)
 * @param i_threshold   Current threshold below which compensation is disabled [A].
 *                      Derive this from your ADC noise floor and current ripple.
 *                      Typical: 2-5% of rated motor current.
 */
__attribute__((always_inline)) static inline float32_t dead_time_compensation(
                                  const float32_t raw_duty, const float32_t i_phase,
                                  const float32_t dt_comp_value, const float32_t i_threshold) {
    /* If current is positive, body diode drops voltage to GND during DT.
       Commanded voltage is effectively reduced → add compensation.
       If current is negative, body diode clamps to VBUS during DT.
       Commanded voltage is effectively increased → subtract compensation. */
    
    if (i_phase > i_threshold) {
        return raw_duty + dt_comp_value; 
    } else if (i_phase < -i_threshold) {
        return raw_duty - dt_comp_value;
    } else {
        /* Intentionally empty for MISRA compliance */
    }
    
    /* Current is near zero-crossing: dead-zone region.
     * Options:
     *   a) No compensation (shown here) — simple, small distortion near zero.
     *   b) Linear interpolation through zero — reduces distortion but
     *      can chatter if ADC noise exceeds the interpolation band.
     *   c) Observer-based: use the dq-frame current command sign
     *      instead of the noisy measured phase current.
     * Choose based on acoustic noise and THD tolerance. */
    return raw_duty; 
}
```

## 5. TIM1 Center-Aligned PWM Initialization

This is the core timer setup for 3-phase PWM generation. Use CubeMX to generate the initial pin/clock configuration, then migrate to VSCode + CMake for daily development.

```c
/**
 * @brief Initialize TIM1 for center-aligned 3-phase PWM with complementary outputs.
 *        Generates ADC trigger via TRGO2 using CCR4.
 *
 * @param pwm_freq_hz    Desired PWM frequency (e.g., 20000 for 20kHz)
 * @param dead_time_ns   Desired dead-time (e.g., 500 for 500ns)
 * @param tim_clk_hz     Timer input clock (typically 170 MHz on STM32G4)
 */
void tim1_pwm_init(const uint32_t pwm_freq_hz, const uint16_t dead_time_ns, const uint32_t tim_clk_hz) {

    /* Enable TIM1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    /* --- Counter Configuration --- */
    /* ARR = (Tim_CLK / (2 * PWM_freq)) - 1
     * Factor of 2 because center-aligned counts up AND down per period. */
    const uint32_t arr = (tim_clk_hz / (2u * pwm_freq_hz)) - 1u;
    TIM1->ARR = arr;
    TIM1->PSC = 0u;  /* No prescaler — full resolution */

    /* CR1: Center-Aligned Mode 1 (CMS=01), ARPE=1 (buffered ARR) */
    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;

    /* Repetition Counter: RCR=1 means update event fires every OTHER period. */
    TIM1->RCR = 1u;

    /* --- Output Compare Configuration (Channels 1-3: PWM) --- */
    /* OC1M = PWM Mode 1 (110), preload enabled (OC1PE) */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    /* Initial duty = 50% (safe idle) */
    const uint32_t half_arr = arr >> 1u;
    TIM1->CCR1 = half_arr;
    TIM1->CCR2 = half_arr;
    TIM1->CCR3 = half_arr;

    /* --- Channel 4: ADC Trigger Source --- */
    /* CCR4 sets the sampling instant within the PWM period. */
    TIM1->CCR4 = 1u;  /* Near valley — fine-tune on bench */
    TIM1->CCMR2 |= (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    /* TRGO2 = OC4REF → ADC external trigger source
     * Use read-modify-write to preserve OISx bits set elsewhere. */
    MODIFY_REG(TIM1->CR2, TIM_CR2_MMS2_Msk, (0x7u << TIM_CR2_MMS2_Pos));

    /* --- Output Enable (CCER) --- */
    /* Enable complementary outputs for channels 1-3 */
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE
               | TIM_CCER_CC2E | TIM_CCER_CC2NE
               | TIM_CCER_CC3E | TIM_CCER_CC3NE;

    /* --- Break & Dead-Time (BDTR) --- */
    /* DTG calculation (simplified for short dead-times, DTG[7:5]=0xx):
     * Dead-time = DTG[7:0] × Tdts where Tdts = 1/Tim_CLK */
    const uint8_t dtg = (uint8_t)((uint64_t)dead_time_ns * tim_clk_hz / 1000000000ULL);

    TIM1->BDTR = (uint32_t)(dtg & 0xFFu)
              | TIM_BDTR_BKE           /* Break enable (for COMP → OCP) */
              | TIM_BDTR_BKP           /* Break polarity (match your COMP output) */
              | TIM_BDTR_OSSI          /* Off-state idle: force OIS levels */
              | TIM_BDTR_OSSR;         /* Off-state run: complementary active */

    /* Safe-state: High-Z (all FETs OFF) by default */
    TIM1->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS1N
                 | TIM_CR2_OIS2 | TIM_CR2_OIS2N
                 | TIM_CR2_OIS3 | TIM_CR2_OIS3N);

    /* --- Start Timer --- */
    TIM1->EGR = TIM_EGR_UG;   /* Force update to load shadow registers */
    TIM1->SR  = 0u;           /* Clear any pending flags */
    TIM1->CR1 |= TIM_CR1_CEN; /* Start counting */
}

/**
 * @brief Enable PWM outputs. Call only after all peripherals are configured
 *        and the system is ready to drive the motor.
 */
static inline void foc_enable_outputs(void) {
    TIM1->BDTR |= TIM_BDTR_MOE;
}
```

## 6. ADC Configuration for FOC (Dual Regular Simultaneous)

For 2-shunt or 3-shunt configurations, sample two phase currents simultaneously using ADC1 + ADC2 in dual mode. Bus voltage and temperature can be sampled in the same sequence or via injected channels.

```c
/**
 * @brief Initialize ADC1 + ADC2 for dual regular simultaneous FOC sensing.
 *        Triggered by TIM1_TRGO2 (= OC4REF from Section 5).
 *
 *        ADC1 → Phase A current (+ bus voltage in sequence)
 *        ADC2 → Phase B current (+ temperature in sequence)
 *
 *        DMA transfers results to a buffer for ISR processing.
 */
void adc_foc_init(void) {

    /* Enable ADC1 + ADC2 clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

    /* --- ADC Common: Dual Regular Simultaneous Mode --- */
    /* DUAL[4:0] = 00110 → Regular simultaneous mode */
    ADC12_COMMON->CCR = (6u << ADC_CCR_DUAL_Pos)
                      | (0u << ADC_CCR_DELAY_Pos)   /* No delay between conversions */
                      | ADC_CCR_DMACFG             /* DMA circular mode */
                      | (2u << ADC_CCR_MDMA_Pos);  /* DMA for 12-bit dual: 32-bit word */

    /* --- ADC1 Configuration --- */
    /* Regular sequence: 1 conversion → Phase A current channel */
    ADC1->SQR1 = (0u << ADC_SQR1_L_Pos)           /* 1 conversion in sequence */
               | (PHASE_A_ADC_CH << ADC_SQR1_SQ1_Pos);
    /* Sampling time: choose based on OPAMP settling + source impedance.
     * 12.5 ADC clocks @ 42.5 MHz = ~294 ns — usually adequate for internal OPAMP. */
    /* (Configure SMPRx register for the specific channel) */

    /* External trigger: TIM1_TRGO2 rising edge */
    ADC1->CFGR = ADC_CFGR_EXTEN_0                 /* Rising edge trigger */
               | (0xAu << ADC_CFGR_EXTSEL_Pos);   /* TIM1_TRGO2 (check RM0440 Table) */

    /* --- ADC2 Configuration --- */
    ADC2->SQR1 = (0u << ADC_SQR1_L_Pos)
               | (PHASE_B_ADC_CH << ADC_SQR1_SQ1_Pos);

    /* ADC2 trigger is implicit in dual mode — follows ADC1 trigger */

    /* --- DMA Configuration (DMA1 Channel 1 for ADC1) --- */
    /* DMA transfers the dual ADC result (32-bit: ADC2[31:16] | ADC1[15:0])
     * into a circular buffer. The ADC ISR (or DMA TC interrupt) then
     * unpacks the results and runs the FOC current loop. */

    /* Enable ADCs */
    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;

    /* Start conversion (will be triggered by TIM1_TRGO2) */
    ADC1->CR |= ADC_CR_ADSTART;
}
```

### Bus Voltage & Temperature Sampling

Avoid sampling bus voltage and temperature in the time-critical dual-simultaneous sequence. Instead, use **injected channels** triggered at a lower rate, or sample them during the second half of the PWM period when current readings are not needed:

```c
/**
 * @brief Read bus voltage and temperature via ADC injected conversion.
 *        Triggered at a lower rate (e.g., every 10th PWM cycle via RCR or software).
 */
void adc_read_auxiliary(float32_t *v_bus, float32_t *temperature) {
    /* Start injected conversion on ADC1 (software triggered or secondary timer) */
    ADC1->JSQR = (VBUS_ADC_CH << ADC_JSQR_JSQ1_Pos)
               | (TEMP_ADC_CH << ADC_JSQR_JSQ2_Pos)
               | (1u << ADC_JSQR_JL_Pos);   /* 2 injected conversions */

    ADC1->CR |= ADC_CR_JADSTART;

    /* Results available in ADC1->JDR1 (Vbus) and ADC1->JDR2 (Temp) */
    /* Convert to engineering units using calibration constants */
    *v_bus = (float32_t)ADC1->JDR1 * VBUS_SCALE;
    *temperature = (float32_t)ADC1->JDR2 * TEMP_SCALE + TEMP_OFFSET;
}
```
