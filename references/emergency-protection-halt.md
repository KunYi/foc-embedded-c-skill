# Emergency Protection & Automatic Halt (Reference)

## Overview
Control theory assumes nominal operation. Reality contains sudden sensor failure, cable disconnection, extreme load spikes, and CPU crashes (HardFault). The mark of a professional drive is not how beautifully it spins, but how safely it fails.

## 1. Hardware State vs. Software State
If the MCU enters a `HardFault_Handler()` or the Watchdog triggers a reset, software is DEAD. The PWM pins must autonomously drop to a safe analog state without executing C code.

**STM32G4 BDTR (`Break and Dead-Time Register`) Constraint:**
- Set `BKP` (Break Polarity) to active High/Low depending on your physical external hardware comparator.
- Disable `MOE` (Main Output Enable). When `MOE=0`, `OSSR` (Off-State Selection for Run mode) and `OSSI` (Off-State Selection for Idle mode) dictate pin states.
- **Rule:** Define the hardware fault default state from system hazard analysis. High-Z is a common choice for many faults, but it is not universally safest if uncontrolled regeneration or overspeed can overvoltage the DC bus.

## 2. Choosing a Safe State: High-Z vs. ASC

When you trigger `TIM1_BRK` via COMP (Overcurrent) or software detects overspeed, the drive must halt. How?

### A. High-Z Coasting (All FETs = OFF)
- **Mechanism**: The 6 Gate signals are pulled LOW. Motor phases disconnect. Current stops regenerating actively.
- **Risk**: As the motor coasts down, the rotor's permanent magnets act as a generator. If the motor is spinning fast (Back-EMF > $V_{Bus}$), the phase voltage will forward-bias the body diodes of the High-Side FETs, slamming uncontrolled power straight into the DC Bus capacitors.
- **When to Use**: General faults, short-circuits to Ground, encoder disconnections, when running at speeds where Peak BEMF < $V_{dc\_rating}$.

### B. Active Short Circuit (ASC) / Six-Step Low
- **Mechanism**: All 3 Bottom FETs = ON (100% duty), All 3 Top FETs = OFF.
- **Physics**: The motor is actively short-circuited across the ground rail. The BEMF is dissipated as heat purely in the stator windings and the $R_{ds(on)}$ of the bottom FETs. No voltage pumps back to the DC link!
- **Risk**: Enormous braking torque (which can snap mechanical couplings) and extreme localized heating in the stator.
- **When to Use**: Consider ASC when uncontrolled regeneration or overspeed makes High-Z unsafe, especially in deep field-weakening or high-inertia backdriving scenarios. The correct choice depends on bus-energy absorption capability, mechanical survivability, and gate-driver safe-state behavior.

```c
/**
 * @brief Enters Active Short Circuit (ASC) for emergency FW shutdown.
 */
void enter_emergency_asc(void) {
    /* 1. Disable normal center-aligned PWM updates */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    
    /* 2. Force outputs: High-Side = OFF, Low-Side = ON */
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E); /* Top OFF */
    TIM1->CCER |=  (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE); /* Bot ON */
    
    /* 3. Force duty registers to 100% just in case of mode transition */
    TIM1->CCR1 = TIM1->ARR;
    TIM1->CCR2 = TIM1->ARR;
    TIM1->CCR3 = TIM1->ARR;
    
    /* Latch software fault state */
    g_foc_state = FOC_STATE_FAULT_ASC;
}
```

## 3. Overspeed / Load Loss (Belt Snap)
If the mechanical load suddenly disconnects (e.g. snapped belt in a spindle), the integrating PI speed controller will ramp torque $I_q$ to maximum. The un-loaded rotor will accelerate to destruction speeds in milliseconds.

- **Constraint**: You MUST have a hardware-backed speed limit checker in the fastest loop possible (e.g., inside the Hall or QEP edge-capture ISR, or the Current Loop checking `omega_e`).
- If `mechanic_rpm > MAX_SAFE_RPM`, immediately trigger ASC or High-Z (based on BEMF levels) and shut down the PI integrators to zero.
