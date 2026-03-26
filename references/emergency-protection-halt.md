# Emergency Protection & Automatic Halt (Reference)

## Overview
Control theory assumes nominal operation. Reality contains sudden sensor failure, cable disconnection, extreme load spikes, and CPU crashes (HardFault). The mark of a professional drive is not how beautifully it spins, but how safely it fails.

## 1. Hardware State vs. Software State
If the MCU enters a `HardFault_Handler()` or the Watchdog triggers a reset, software is DEAD. The PWM pins must autonomously drop to a safe analog state without executing C code.

**STM32G4 BDTR (`Break and Dead-Time Register`) — The Last Line of Defence:**

The BDTR register and associated `TIM1_CR2` bits define what happens to the PWM outputs when:
- A hardware break event fires (COMP overcurrent → `TIM_BRK`)
- Software clears `MOE` (Main Output Enable)
- The MCU enters HardFault or resets

**Key Configuration Bits:**

| Bit | Purpose | Safety Rule |
|-----|---------|-------------|
| `MOE` | Master Output Enable — gates all PWM outputs | When `MOE=0`, outputs go to the state defined by `OSSI`/`OSSR` and `OISx`/`OISxN` |
| `OSSI` | Off-State Selection for Idle mode | Set `OSSI=1` to force outputs to `OISx`/`OISxN` values when `MOE=0` |
| `OSSR` | Off-State Selection for Run mode | Set `OSSR=1` to maintain complementary dead-time behavior during run mode |
| `BKE` | Break Enable | Must be `1` to enable hardware break input |
| `BKP` | Break Polarity | Match your external COMP/fault signal polarity |
| `AOE` | Automatic Output Enable | Usually `0` for motor drives — require explicit software re-enable after fault |
| `OISx` / `OISxN` | Output Idle State for main / complementary channels | **These define the safe-state pin levels when MOE=0** |

**Critical Rule:** The safe-state output pattern is defined by `OISx` and `OISxN` bits in `TIM1_CR2`, NOT by `CCRx` values. Configure these during initialization based on your system hazard analysis.

```c
/**
 * @brief Configure TIM1 BDTR for safe-state behavior.
 *        Call once during initialization, before enabling PWM outputs.
 *
 *        This example configures for High-Z default (all FETs OFF on fault).
 *        Change OISx/OISxN pattern for ASC if required.
 */
void tim1_configure_break_safe_state(uint16_t dead_time_ns, uint32_t tim_clk_hz) {

    /* 1. Calculate DTG field from desired dead-time and timer clock.
          The DTG encoding is non-linear — refer to RM0440 for the exact formula.
          This is a simplified example for short dead-times (DTG[7:5] = 0xx). */
    uint8_t dtg = (uint8_t)((dead_time_ns * (tim_clk_hz / 1000000UL)) / 1000UL);

    /* 2. Configure BDTR:
          - BKE=1: Enable break input
          - BKP=1: Break input active high (match your COMP output polarity)
          - OSSI=1: Force OISx/OISxN states when MOE=0
          - OSSR=1: Enable complementary output during run
          - AOE=0: Do NOT auto-re-enable after fault (require software clear)
          - MOE=0: Start with outputs disabled */
    TIM1->BDTR = (dtg & 0xFFu)
              | TIM_BDTR_BKE       /* Break enable */
              | TIM_BDTR_BKP       /* Break polarity = active high */
              | TIM_BDTR_OSSI      /* Off-state idle: force OIS levels */
              | TIM_BDTR_OSSR;     /* Off-state run: complementary active */
    /* AOE=0 (default), MOE=0 (default) — outputs start disabled */

    /* 3. Define the safe-state output pattern in TIM1_CR2.
          For HIGH-Z (all FETs OFF): OISx=0, OISxN=0 for all channels.
          This is the reset default, shown explicitly for clarity. */
    TIM1->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS1N
                 | TIM_CR2_OIS2 | TIM_CR2_OIS2N
                 | TIM_CR2_OIS3 | TIM_CR2_OIS3N);
}
```

## 2. Choosing a Safe State: High-Z vs. ASC

When you trigger `TIM1_BRK` via COMP (Overcurrent) or software detects overspeed, the drive must halt. How?

### A. High-Z Coasting (All FETs = OFF)
- **Mechanism**: The 6 Gate signals are pulled LOW. Motor phases disconnect. Current stops regenerating actively.
- **STM32G4 Config**: `OIS1=0, OIS1N=0, OIS2=0, OIS2N=0, OIS3=0, OIS3N=0` — this is the default.
- **Risk**: As the motor coasts down, the rotor's permanent magnets act as a generator. If the motor is spinning fast (Back-EMF > $V_{Bus}$), the phase voltage will forward-bias the body diodes of the High-Side FETs, slamming uncontrolled power straight into the DC Bus capacitors.
- **When to Use**: General faults, short-circuits to Ground, encoder disconnections, when running at speeds where Peak BEMF < $V_{dc\_rating}$.

### B. Active Short Circuit (ASC) / Three Low-Side ON
- **Mechanism**: All 3 Bottom FETs = ON (forced active), All 3 Top FETs = OFF (forced inactive).
- **STM32G4 Config**: `OIS1=0, OIS1N=1, OIS2=0, OIS2N=1, OIS3=0, OIS3N=1` in `TIM1_CR2`.
- **Physics**: The motor is actively short-circuited across the ground rail. The BEMF is dissipated as heat purely in the stator windings and the $R_{ds(on)}$ of the bottom FETs. No voltage pumps back to the DC link.
- **Risk**: Enormous braking torque (which can snap mechanical couplings) and extreme localized heating in the stator.
- **When to Use**: Consider ASC when uncontrolled regeneration or overspeed makes High-Z unsafe, especially in deep field-weakening or high-inertia backdriving scenarios. The correct choice depends on bus-energy absorption capability, mechanical survivability, and gate-driver safe-state behavior.

### Implementing ASC via Software Trigger

```c
/**
 * @brief Enters Active Short Circuit (ASC) for emergency shutdown.
 *
 *        Uses STM32G4 forced output mode on the timer channels to guarantee
 *        low-side ON, high-side OFF — independent of CCR values or timer state.
 *
 *        Method: Set OISxN=1 (low-side forced active on break) then
 *        clear MOE to trigger the break idle state.
 *        The timer keeps running (CEN=1) so that ADC sync and current
 *        monitoring remain functional for fault diagnostics.
 */
void enter_emergency_asc(void) {

    /* 1. Set idle-state pattern for ASC:
          OISx  = 0 (main/high-side channels → inactive/LOW on break)
          OISxN = 1 (complementary/low-side channels → active/HIGH on break)

          These bits take effect when MOE transitions from 1→0. */
    uint32_t cr2 = TIM1->CR2;
    cr2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3);    /* High-side OFF */
    cr2 |=  (TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N);  /* Low-side ON  */
    TIM1->CR2 = cr2;

    /* 2. Clear MOE to force outputs into the OISx/OISxN idle state.
          With OSSI=1 (configured at init), outputs are actively driven
          to the defined idle levels, not tri-stated. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    /* 3. Timer continues running (CEN remains set).
          ADC triggering via TRGO remains active for fault current monitoring.
          Do NOT clear CEN — a stopped timer with OSSI might behave
          differently on some STM32 variants. */

    /* 4. Latch software fault state */
    g_foc_state = FOC_STATE_FAULT_ASC;
}

/**
 * @brief Enters High-Z coasting for emergency shutdown.
 */
void enter_emergency_highz(void) {

    /* OISx=0, OISxN=0 for all channels → all FETs OFF */
    TIM1->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS1N
                 | TIM_CR2_OIS2 | TIM_CR2_OIS2N
                 | TIM_CR2_OIS3 | TIM_CR2_OIS3N);

    TIM1->BDTR &= ~TIM_BDTR_MOE;

    g_foc_state = FOC_STATE_FAULT_HIGHZ;
}
```

### Safe-State Decision Tree

```
Fault Detected
  ├── Is motor in Field Weakening (high speed, |BEMF| approaching Vdc)?
  │     ├── YES → ASC (prevent regenerative overvoltage)
  │     └── NO  → Is it a phase-to-ground short circuit?
  │                 ├── YES → High-Z (ASC would feed the fault)
  │                 └── NO  → High-Z (safe default for most faults)
  └── Is the DC bus near OVP threshold?
        ├── YES → ASC (absorb energy in windings, not capacitors)
        └── NO  → High-Z
```

## 3. Overspeed / Load Loss (Belt Snap)
If the mechanical load suddenly disconnects (e.g., snapped belt in a spindle), the integrating PI speed controller will ramp torque $I_q$ to maximum. The un-loaded rotor will accelerate to destruction speeds in milliseconds.

- **Constraint**: You MUST have a hardware-backed speed limit checker in the fastest loop possible (e.g., inside the Hall or QEP edge-capture ISR, or the Current Loop checking `omega_e`).
- If `mechanic_rpm > MAX_SAFE_RPM`, immediately trigger ASC or High-Z (based on BEMF levels) and shut down the PI integrators to zero.

## 4. HardFault Handler — Last Resort

When the MCU enters `HardFault_Handler()`, the C runtime may be corrupted. Do not attempt complex recovery.

```c
/**
 * @brief Minimal HardFault handler for motor drive safety.
 *        Only touches hardware registers — no heap, no stack frames.
 */
void HardFault_Handler(void) {
    /* Force MOE=0 immediately. OSSI + OISx/OISxN (configured at init)
       will drive outputs to the pre-defined safe state. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    /* Optional: signal fault via a dedicated GPIO (directly, no HAL) */
    GPIOB->BSRR = GPIO_BSRR_BS14;  /* Example: set PB14 fault LED */

    /* Infinite loop — wait for watchdog reset */
    while (1) { __NOP(); }
}
```
