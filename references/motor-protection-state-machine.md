# Motor Protection & State Machine (Reference)

## Overview
FOC software must rigidly orchestrate the transition from a dead stop to a spinning rotor while maintaining hardware safety. 

Treat the sequence and thresholds in this document as reference patterns, not universal constants. Current levels, dwell times, and trip thresholds must be scaled to the motor's thermal limits, inverter current capability, bus voltage, and mechanical risk.

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
- Apply a D-axis alignment current and set `I_q_ref = 0A`, then hold a fixed electrical angle such as $\theta = 0$. Start from the minimum current that reliably pulls the rotor into a known position.
- A common starting point is a modest fraction of rated current, then increase only as needed. Avoid using a hard-coded alignment current across all motors.
- Hold alignment only as long as needed to settle the rotor while staying within thermal limits. The correct dwell may range from a few milliseconds to hundreds of milliseconds depending on inertia, friction, and motor heating.
- Reset the TIM QEP counter to 0. Transition to `FOC_STATE_CLOSED_LOOP`.

## 2. Fault Protection Architectures

### A. Overcurrent (OCP)
- **Hardware Layer**: Handled exclusively by `COMP` to `TIM_BRK` (zero software latency).
- **Software Layer**: Read the status register in the background loop to confirm `TIM_BRK` fired. If fired, latch the software state to `FOC_STATE_FAULT` to disable high-level speed targets.

### B. DC-Bus Overvoltage (OVP) / Regenerative Braking
- Quickly decelerating a heavy inertia load forces the motor into a generator state ($I_q < 0$). This pumps flyback current directly into the DC Bus link capacitors.
- **Rule**: Define a bus overvoltage threshold with adequate margin below absolute capacitor, gate-driver, and inverter limits. If `V_bus` crosses that threshold, the software MUST engage the configured mitigation path such as a bleeder circuit, brake resistor, torque-command reduction, or a controlled safe-stop strategy.

### C. Stall / Blocked Rotor Detection
- If the load completely jams, the current will saturate at maximum limits, but the rotor $\omega$ drops to zero. 
- In sensorless mode, zero speed = no BEMF = the SMO observer diverges wildly.
- **Rule**: Detect stall from a sustained mismatch between torque-producing current and achieved speed or position. Use thresholds and time windows derived from the motor thermal time constant, expected load transients, and sensor resolution rather than relying on a universal current or RPM value.

## 3. The "Catch-Spin" / Flying Start
If the inverter reboots while the motor is already coasting at 3000 RPM (e.g., fan blade caught in wind), instantly firing a zero-angle PWM vector will cause a massive structural shock and overcurrent explosion.
- You must run the SMO Observer silently in the background (PWMs OFF, tracking ADC Back-EMF vectors) until the PLL locks onto the physical $\theta$ and $\omega$.
- Only enable the PWM outputs using the pre-locked $\theta$ angle.
