# BLDC Six-Step Trapezoidal Commutation (Reference)

## Overview

Six-step (trapezoidal) commutation is the simplest and most common control method for Brushless DC (BLDC) motors with trapezoidal back-EMF profiles. It is fundamentally different from sinusoidal FOC:

| Aspect | Six-Step | Sinusoidal FOC |
|--------|----------|----------------|
| BEMF shape | Trapezoidal | Sinusoidal |
| Commutation | 6 discrete states / revolution | Continuous |
| Current waveform | Quasi-square | Sinusoidal |
| Torque ripple | Higher (~15% inherent) | Very low |
| MCU cost | Low (simple timer + comparators) | Higher (fast ADC + trig) |
| Noise | Higher audible noise | Quieter |
| Typical use | Fans, pumps, e-bikes, ESC | Servo, precision, EV traction |

**When to use Six-Step**: Cost-sensitive applications where torque ripple and acoustic noise are acceptable, or when the motor has a strongly trapezoidal BEMF and does not benefit from sinusoidal drive.

## 1. Hall Sensor → Commutation Table

Three Hall sensors placed 120° apart provide 6 valid states per electrical revolution. Each state directly maps to which pair of FETs to drive.

```c
/**
 * @brief Hall sensor state to commutation pattern mapping.
 *
 * Hall state encoding: H3:H2:H1 (3-bit, values 1-6 are valid, 0 and 7 are fault)
 *
 * Each entry defines: which high-side FET and which low-side FET to activate.
 * The third phase is left floating (high-impedance).
 *
 * This table must be calibrated to your specific motor's Hall sensor placement
 * and winding direction. Swap entries or reverse table order for opposite rotation.
 */
typedef struct {
    uint8_t high_phase;   /* 0=none, 1=U, 2=V, 3=W */
    uint8_t low_phase;    /* 0=none, 1=U, 2=V, 3=W */
} commutation_step_t;

/* Standard 120° Hall placement — CW rotation.
 * Index 0 and 7 are invalid Hall states (fault). */
static const commutation_step_t comm_table[8] = {
    /*  H3H2H1  High   Low     Active phases  */
    {0, 0},  /* 000 — INVALID (sensor fault) */
    {3, 2},  /* 001 — W→V */
    {1, 3},  /* 010 — U→W */
    {1, 2},  /* 011 — U→V */
    {2, 1},  /* 100 — V→U */
    {3, 1},  /* 101 — W→U */
    {2, 3},  /* 110 — V→W */
    {0, 0},  /* 111 — INVALID (sensor fault) */
};
```

### STM32G4 Implementation with TIM1 COM Event

STM32 advanced timers support **automatic commutation** via the COM (Commutation) event. This is the production approach — it ensures the new PWM pattern is applied synchronously with the timer update, preventing glitches.

```c
/**
 * @brief Apply commutation step to TIM1 outputs.
 *        Uses forced output mode for the floating phase and PWM for the active phases.
 *
 *        CCER controls which channels are enabled.
 *        OC1M/OC2M/OC3M control the output mode per channel.
 *
 *        Call this from the Hall sensor edge interrupt. The actual output change
 *        is deferred to the next TIM1 COM event for glitch-free switching.
 */
void bldc_apply_commutation(uint8_t hall_state, float32_t duty) {

    const commutation_step_t *step = &comm_table[hall_state];

    /* Validate Hall state */
    if (step->high_phase == 0 || step->low_phase == 0) {
        /* Invalid Hall state — fault */
        TIM1->BDTR &= ~TIM_BDTR_MOE;  /* Disable all outputs */
        g_foc_state = FOC_STATE_FAULT;
        return;
    }

    uint16_t ccr_val = (uint16_t)(duty * (float32_t)TIM1->ARR);

    /* Start with all channels disabled */
    uint32_t ccer = 0;
    uint32_t ccmr1 = TIM1->CCMR1 & ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
    uint32_t ccmr2 = TIM1->CCMR2 & ~(TIM_CCMR2_OC3M_Msk);

    /* Set active high-side channel: PWM mode 1, output enabled */
    /* Set active low-side channel: Forced active (100% ON), N-output enabled */
    /* Floating channel: outputs disabled (Hi-Z) */

    for (uint8_t phase = 1; phase <= 3; phase++) {
        if (phase == step->high_phase) {
            /* PWM on main channel (high-side via gate driver) */
            switch (phase) {
                case 1:
                    ccmr1 |= (6u << TIM_CCMR1_OC1M_Pos);  /* PWM Mode 1 */
                    ccer |= TIM_CCER_CC1E;
                    TIM1->CCR1 = ccr_val;
                    break;
                case 2:
                    ccmr1 |= (6u << TIM_CCMR1_OC2M_Pos);
                    ccer |= TIM_CCER_CC2E;
                    TIM1->CCR2 = ccr_val;
                    break;
                case 3:
                    ccmr2 |= (6u << TIM_CCMR2_OC3M_Pos);
                    ccer |= TIM_CCER_CC3E;
                    TIM1->CCR3 = ccr_val;
                    break;
            }
        } else if (phase == step->low_phase) {
            /* Force low-side ON (complementary output active, main forced inactive) */
            switch (phase) {
                case 1:
                    ccmr1 |= (4u << TIM_CCMR1_OC1M_Pos);  /* Forced inactive (high OFF) */
                    ccer |= TIM_CCER_CC1NE;                 /* Low-side N-channel ON */
                    break;
                case 2:
                    ccmr1 |= (4u << TIM_CCMR1_OC2M_Pos);
                    ccer |= TIM_CCER_CC2NE;
                    break;
                case 3:
                    ccmr2 |= (4u << TIM_CCMR2_OC3M_Pos);
                    ccer |= TIM_CCER_CC3NE;
                    break;
            }
        }
        /* Floating phase: no bits set → Hi-Z */
    }

    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = ccmr2;
    TIM1->CCER  = ccer;

    /* Trigger COM event to apply the new configuration synchronously */
    TIM1->EGR = TIM_EGR_COMG;
}
```

## 2. Sensorless Six-Step (BEMF Zero-Crossing)

When Hall sensors are not available, the back-EMF zero-crossing on the floating phase determines the commutation instant. The floating phase BEMF crosses the virtual neutral point midway between two commutation events.

### Principle
- During each commutation step, one phase is floating (Hi-Z).
- The floating phase BEMF transitions from positive to negative (or vice versa) as the rotor passes through the commutation point.
- Detect this zero-crossing using an analog comparator or ADC sampling.
- After detecting the crossing, wait 30° electrical (half of 60° step) before commutating.

### STM32G4 Comparator-Based Detection

```c
/**
 * @brief BEMF zero-crossing detection using STM32G4 internal COMP.
 *
 * The floating phase is routed to the COMP positive input.
 * The virtual neutral point (or Vbus/2) is the COMP negative reference.
 *
 * When COMP output transitions, a timer input capture stamps the event.
 * The 30° advance delay is computed from the last commutation period.
 */

/* Called from COMP IRQ when zero-crossing is detected */
void COMP_IRQHandler(void) {
    uint32_t capture_time = TIM2->CCR1;  /* Input capture timestamp */

    /* Calculate time since last commutation */
    uint32_t period = capture_time - g_last_comm_time;

    /* 30° delay = half of one 60° step period
     * Next commutation = zero_crossing + period/2 */
    uint32_t delay_30deg = period >> 1;

    /* Schedule next commutation via compare interrupt */
    TIM2->CCR2 = capture_time + delay_30deg;
    TIM2->DIER |= TIM_DIER_CC2IE;  /* Enable compare interrupt */

    /* Acknowledge COMP interrupt */
    EXTI->PR1 = EXTI_PR1_PIF21;  /* COMP1 mapped to EXTI21 on STM32G4 */
}

/* TIM2_CC2 fires at the 30° delay point → commutate */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_CC2IF) {
        TIM2->SR = ~TIM_SR_CC2IF;

        g_bldc_step = (g_bldc_step % 6) + 1;
        bldc_apply_commutation(g_bldc_step, g_duty);
        g_last_comm_time = TIM2->CCR2;
    }
}
```

### Startup Without Sensors
At standstill, there is no BEMF. A blind startup sequence is required:
1. **Alignment**: Apply a known vector to pull the rotor to a known position (same as FOC alignment).
2. **Forced commutation**: Step through the commutation table at a fixed, slowly increasing frequency until enough BEMF is generated for zero-crossing detection.
3. **Transition to sensorless**: Once the first valid zero-crossing is detected, switch to BEMF-based commutation timing.

## 3. Advance Angle / Timing Compensation

At high speeds, the commutation-to-effect delay (gate driver propagation, current rise time, winding inductance L/R time constant) causes the current waveform to lag behind the ideal position. This reduces torque and efficiency.

**Advance commutation**: Shift the commutation instant earlier by a fixed or speed-proportional angle. Production drives typically use:

$$\theta_{advance} = K_{adv} \times \omega_e$$

where $K_{adv}$ is tuned per motor to maximize efficiency at the target operating speed.

## 4. When to Transition from Six-Step to Sinusoidal FOC

Consider upgrading from six-step to sinusoidal FOC when:
- Torque ripple causes mechanical vibration or acoustic noise complaints
- Efficiency at partial load matters (sinusoidal has lower copper loss for the same torque)
- Speed range extension via field weakening is needed
- The motor has a sinusoidal BEMF profile (many "BLDC" motors are actually PMSM)
- Precise position or speed tracking is required

Consider staying with six-step when:
- Cost and MCU simplicity dominate
- The motor has a genuinely trapezoidal BEMF (rare in modern motors)
- High-speed fan/pump applications where ripple is mechanically filtered
- The existing control hardware lacks the ADC bandwidth for FOC
