# Current Sensing Topologies & ADC Guidelines (Reference)

## Overview
FOC requires precise phase current recreation. The shunt configuration dictates not just the ADC triggering logic and SVPWM limits, but strictly enforces layout safety and hardware protection mechanisms. **Any error in analog filtering or PCB routing here renders the software useless.**

Use the numeric examples in this document as starting points only. Final values depend on PWM frequency, shunt value, op-amp bandwidth, ADC acquisition time, board parasitics, and acceptable phase delay in the control loop.

## 1. Shunt Resistor Physical Selection & PCB Layout

**The Simulation Trap:** 
SIL assumes a shunt is a pure resistor. In reality, a physical SMD Shunt has Parasitic Inductance (ESL). High $di/dt$ switching currents cause $V_{noise} = ESL \times \frac{di}{dt}$ voltage spikes across the shunt, completely overriding your $I \times R$ signal.

- **Resistor Choice**: Do not pick arbitrary current-sense parts without checking pulse handling, inductance, tolerance, and thermal drift. Strongly prefer low-ESL shunts and validate the parasitic behavior against your actual switching edges.
- **Wattage & Resolution**: Trade-off. Higher resistance = better ADC clarity but hotter boards. Aim to utilize 80% of your OPAMP range at absolute peak hardware limit current to maximize dynamic range.
- **Kelvin Connection (4-Wire Routing)**: The PCB traces fetching the Shunt voltage MUST route from the very inside pads of the resistor. If traces grab from the outside, solder resistance ruins calibration. 
- **Trace Parallelism**: The differential traces (`Shunt+`, `Shunt-`) must route perfectly parallel and close together to cancel out EMI common-mode noise. **Keep them entirely isolated from Phase-node polygons.**

## 2. Noise Handling: Anti-Aliasing RC Filters and the Phase-Shift Dilemma

You MUST place a hardware low-pass RC filter before the OPAMP to smooth out PWM switching noise.
- **The Dilemma**: 
  - Too small $R/C$: High frequency switching noise hits the ADC. 
  - Too large $R/C$: Delays the current signal! If your current loops delays by $5\mu s$ on a $20kHz$ loop, the d-q decoupling logic applies the counter-voltage to the WRONG electrical angle, causing instability.
- **Rule of Thumb Constraint**: Start with a cut-off frequency high enough that analog delay remains a small fraction of the current-loop sample interval, then verify with scope data and closed-loop behavior. A multiple of the PWM frequency is a useful starting heuristic, not a substitute for measurement.

## 3. Shunt Topologies 

### A. Three-Shunt Phase Sensing (Low-Side)
- One shunt on each of the three low-side FET source/ground returns.
- **Advantage**: Always captures current accurately as long as the bottom FET is conducting.
- **Software Action**: You only need 2 ADC readings (by Kirchhoff's $I_u + I_v + I_w = 0$), but if PWM duty approaches the region where valid sampling windows collapse, software must either clamp duty, alter modulation, or reconstruct currents according to the active sector. The safe duty ceiling is hardware-dependent.
- **STM32G4**: Use ADC1 + ADC2 in dual regular simultaneous mode for two channels, or injected channels triggered by TIM1_TRGO. The third phase is computed via KCL.

### B. Two-Shunt Sensing (Low-Side)
- Shunts on 2 of 3 low-side legs (commonly Phase A and Phase B, or Phase A and Phase C). This is the **most common cost-optimized topology** used in production drives (e.g., ST B-G431B-ESC1 Discovery Kit).
- **Advantage**: Saves one shunt resistor, one op-amp channel, and simplifies PCB. KCL reconstructs the missing phase: $I_C = -(I_A + I_B)$.
- **Sector-Dependent Blind Zone**: Like 3-shunt, when the bottom FET of a measured phase has very short on-time, that ADC reading becomes invalid. With only 2 shunts, you lose the **redundancy** that 3-shunt provides — there is always one phase you cannot directly verify.

```c
/**
 * @brief Two-shunt current reconstruction with sector-awareness.
 *        Measures Phase A and Phase B; reconstructs Phase C via KCL.
 *
 *        In sectors where Phase A or Phase B bottom-FET on-time is too short
 *        for a valid ADC reading, the firmware must use the other measured
 *        phase + the previously computed KCL value, or shift the PWM pattern.
 */
__attribute__((always_inline)) static inline void two_shunt_reconstruct(
                            const float32_t adc_ia, const float32_t adc_ib, const uint8_t svpwm_sector,
                            float32_t * const ia_out, float32_t * const ib_out, float32_t * const ic_out) {

    /* Default: both ADC readings are valid */
    const float32_t ia = adc_ia;
    const float32_t ib = adc_ib;
    const float32_t ic = -(ia + ib);

    /* Sector-dependent validity check:
     * In certain sectors, one of the measured phases has very short
     * low-side on-time. When this happens, that ADC reading is unreliable.
     *
     * Example for shunts on Phase A and Phase B:
     *   Sector 1,6: Phase A has highest duty → Ia may be invalid
     *   Sector 2,3: Phase B has highest duty → Ib may be invalid
     *   Sector 4,5: Phase C has highest duty → both Ia, Ib are valid
     *
     * When one reading is invalid, reconstruct it from KCL:
     *   Invalid Ia: Ia = -(Ib + Ic_prev)  (use last known Ic)
     *   Invalid Ib: Ib = -(Ia + Ic_prev)
     *
     * Better approach: enforce minimum pulse width on the measured phases
     * BEFORE generating PWM, ensuring valid ADC reads across all sectors.
     */

    *ia_out = ia;
    *ib_out = ib;
    *ic_out = ic;
}

/**
 * @brief Clamps the SVPWM duty cycles to guarantee minimum ADC sampling time.
 *        Required for 2-shunt and 3-shunt topologies to avoid blind zones.
 *        Run this AFTER svpwm_generate() and BEFORE loading timer CCRs.
 *
 * @param duty_u, duty_v, duty_w  Pointers to the calculated duty cycles (0.0 to 1.0)
 * @param min_duty                Minimum duty cycle required by hardware (e.g., 0.05f for 5%)
 * @param max_duty                Maximum duty cycle (e.g., 0.95f for 95%)
 */
__attribute__((always_inline)) static inline void svpwm_clamp_duty(
    float32_t * const duty_u, float32_t * const duty_v, float32_t * const duty_w,
    const float32_t min_duty, const float32_t max_duty) {

    /* Clamp Phase U */
    if (*duty_u < min_duty) {
        *duty_u = min_duty;
    } else if (*duty_u > max_duty) {
        *duty_u = max_duty;
    } else {
        /* MISRA C: Empty else branch for documented intentional fallthrough */
    }

    /* Clamp Phase V */
    if (*duty_v < min_duty) {
        *duty_v = min_duty;
    } else if (*duty_v > max_duty) {
        *duty_v = max_duty;
    } else {
        /* Intentional fallthrough */
    }

    /* Clamp Phase W */
    if (*duty_w < min_duty) {
        *duty_w = min_duty;
    } else if (*duty_w > max_duty) {
        *duty_w = max_duty;
    } else {
        /* Intentional fallthrough */
    }
}
```

- **Minimum Pulse Width**: Production firmware typically enforces a minimum duty on the measured phase legs. On STM32G4, this means clamping `TIM1->CCRx` to keep the bottom-FET on-time ≥ (op-amp settling + ADC sample time + margin). The clamp value is hardware-dependent.
- **When to choose 2-shunt over 3-shunt**: BOM cost pressure, compact PCB space, or when using the STM32G4 internal OPAMPs (only 3 available on some variants, and one is needed for bus voltage divider scaling).

### C. Single-Shunt Sensing (DC-Link)
- One shunt on the main ground return of the inverter.
- **Advantage**: Cheapest, forces all currents through one node.
- **Pain Point**: Recreating the 3 phases mathematically requires parsing the SVPWM sectors into 2 active vectors and sampling TWICE per PWM period in tiny windows between state changes.
- **Asymmetric PWM Constraint**: If an active state is too short for amplifier settling and ADC conversion, software MUST alter the modulation pattern to create a valid sample window. The minimum usable window is set by the analog front-end and ADC timing, so derive it from your hardware rather than assuming a universal microsecond threshold.
- **Engineering Trade-off**: Single-shunt sensing usually increases modulation and reconstruction complexity substantially. Use it deliberately when BOM, mechanics, or integration constraints justify that complexity.

### D. Inline Sensing (Phase Wire)
- Inductive or Hall-effect sensors (e.g., Allegro ACS) placed right on the phase output wire.
- **Advantage**: Zero blind spots. Independent of PWM state.
- **Pain Point**: Expensive. Highly susceptible to common-mode voltage transients (dv/dt jumping from 0 to bus voltage). Wait until the ringing stops before triggering the ADC.

## 4. Topology Selection Guide

| Criteria | 3-Shunt | 2-Shunt | 1-Shunt | Inline |
|----------|---------|---------|---------|--------|
| BOM Cost | $$$ | $$ | $ | $$$$ |
| Reconstruction complexity | Low | Medium | High | None |
| Blind zones | Rare (redundancy) | Some sectors | Many sectors | None |
| Modulation constraints | Minor | Moderate | Significant | None |
| Used by | High-end servo | Consumer drives, ESC | Ultra-low-cost | Industrial servo |
| STM32G4 fit | ADC1+ADC2+ADC3 | ADC1+ADC2 | Single ADC + DMA | Any ADC |
