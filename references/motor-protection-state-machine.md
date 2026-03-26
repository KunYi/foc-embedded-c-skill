# Motor Protection & State Machine (Reference)

## Overview
FOC software must rigidly orchestrate the transition from a dead stop to a spinning rotor while maintaining hardware safety. 

## 1. FOC Global State Machine

```c
typedef enum {
    FOC_STATE_STANDBY = 0, /* PWMs OFF, checking faults */
    FOC_STATE_ALIGNMENT,   /* Forcing D-axis current to lock rotor (Encoder init) */
    FOC_STATE_OPEN_LOOP,   /* Spinning rotor blindly to build BEMF for SMO */
    FOC_STATE_CLOSED_LOOP, /* Full Sensorless/Sensor FOC Torque/Speed active */
    FOC_STATE_FAULT        /* Hardware trip or software protection engaged */
} foc_state_t;
```

### Alignment (Open-Loop Lock)
For incremental encoders (QEP), the system powers up blind. 
- You MUST force the state machine into `FOC_STATE_ALIGNMENT`.
- Apply `I_d_ref = 5.0A` (Rated limit) and `I_q_ref = 0A`. Set angle $\theta = 0$.
- Hold this for $< 500ms$ (to avoid thermal overload on pure DC coils) to drag the rotor physically to 0 deg.
- Reset the TIM QEP counter to 0. Transition to `FOC_STATE_CLOSED_LOOP`.

## 2. Fault Protection Architectures

### A. Overcurrent (OCP)
- **Hardware Layer**: Handled exclusively by `COMP` to `TIM_BRK` (zero software latency).
- **Software Layer**: Read the status register in the background loop to confirm `TIM_BRK` fired. If fired, latch the software state to `FOC_STATE_FAULT` to disable high-level speed targets.

### B. DC-Bus Overvoltage (OVP) / Regenerative Braking
- Quickly decelerating a heavy inertia load forces the motor into a generator state ($I_q < 0$). This pumps flyback current directly into the DC Bus link capacitors.
- **Rule**: If $V_{bus}$ exceeds 90% of capacitor rating, the software MUST forcibly inject a PWM duty cycle into a dedicated Bleeder Circuit / Brake Resistor, or dynamically drop $I_q$ torque commands to limit regenerative power.

### C. Stall / Blocked Rotor Detection
- If the load completely jams, the current will saturate at maximum limits, but the rotor $\omega$ drops to zero. 
- In sensorless mode, zero speed = no BEMF = the SMO observer diverges wildly.
- **Rule**: If `I_q > Max` for `T > 2 seconds` AND `Speed < 10 RPM`, transition to `FOC_STATE_FAULT` immediately to prevent thermal runaway.

## 3. The "Catch-Spin" / Flying Start
If the inverter reboots while the motor is already coasting at 3000 RPM (e.g., fan blade caught in wind), instantly firing a zero-angle PWM vector will cause a massive structural shock and overcurrent explosion.
- You must run the SMO Observer silently in the background (PWMs OFF, tracking ADC Back-EMF vectors) until the PLL locks onto the physical $\theta$ and $\omega$.
- Only enable the PWM outputs using the pre-locked $\theta$ angle.
