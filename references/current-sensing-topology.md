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

### A. Three-Shunt Phase Sensing
- Directly monitors U, V, and W phase leg currents. 
- **Advantage**: Always captures current accurately as long as the bottom FET is conducting.
- **Software Action**: You only need 2 ADC readings (by Kirchhoff's $I_u + I_v + I_w = 0$), but if PWM duty approaches the region where valid sampling windows collapse, software must either clamp duty, alter modulation, or reconstruct currents according to the active sector. The safe duty ceiling is hardware-dependent.

### B. Single-Shunt Sensing (DC-Link)
- One shunt on the main ground return of the inverter.
- **Advantage**: Cheap, forces all currents through one node.
- **Pain Point**: Recreating the 3 phases mathematically requires parsing the SVPWM sectors into 2 active vectors and sampling TWICE per PWM period in tiny windows between state changes.
- **Asymmetric PWM Constraint**: If an active state is too short for amplifier settling and ADC conversion, software MUST alter the modulation pattern to create a valid sample window. The minimum usable window is set by the analog front-end and ADC timing, so derive it from your hardware rather than assuming a universal microsecond threshold.
- **Engineering Trade-off**: Single-shunt sensing usually increases modulation and reconstruction complexity substantially. Use it deliberately when BOM, mechanics, or integration constraints justify that complexity.

### C. Inline Sensing (Phase Wire)
- Inductive or Hall-effect sensors (e.g., Allegro ACS) placed right on the phase output wire.
- **Advantage**: Zero blind spots. Independent of PWM state.
- **Pain Point**: Expensive. Highly susceptible to common-mode voltage transients (dv/dt jumping from 0 to bus voltage). Wait until the ringing stops before triggering the ADC.
