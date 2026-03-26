# Current Sensing Topologies & ADC Guidelines (Reference)

## Overview
FOC requires precise phase current recreation. The shunt configuration dictates not just the ADC triggering logic and SVPWM limits, but strictly enforces layout safety and hardware protection mechanisms. **Any error in analog filtering or PCB routing here renders the software useless.**

## 1. Shunt Resistor Physical Selection & PCB Layout

**The Simulation Trap:** 
SIL assumes a shunt is a pure resistor. In reality, a physical SMD Shunt has Parasitic Inductance (ESL). High $di/dt$ switching currents cause $V_{noise} = ESL \times \frac{di}{dt}$ voltage spikes across the shunt, completely overriding your $I \times R$ signal.

- **Resistor Choice**: Do not pick generic ceramic resistors. You MUST use **low-ESL metallic foil shunts** (e.g. < 1nH).
- **Wattage & Resolution**: Trade-off. Higher resistance = better ADC clarity but hotter boards. Aim to utilize 80% of your OPAMP range at absolute peak hardware limit current to maximize dynamic range.
- **Kelvin Connection (4-Wire Routing)**: The PCB traces fetching the Shunt voltage MUST route from the very inside pads of the resistor. If traces grab from the outside, solder resistance ruins calibration. 
- **Trace Parallelism**: The differential traces (`Shunt+`, `Shunt-`) must route perfectly parallel and close together to cancel out EMI common-mode noise. **Keep them entirely isolated from Phase-node polygons.**

## 2. Noise Handling: Anti-Aliasing RC Filters and the Phase-Shift Dilemma

You MUST place a hardware low-pass RC filter before the OPAMP to smooth out PWM switching noise.
- **The Dilemma**: 
  - Too small $R/C$: High frequency switching noise hits the ADC. 
  - Too large $R/C$: Delays the current signal! If your current loops delays by $5\mu s$ on a $20kHz$ loop, the d-q decoupling logic applies the counter-voltage to the WRONG electrical angle, causing instability.
- **Rule of Thumb Constraint**: Ensure the Cut-off Frequency $f_c = \frac{1}{2\pi R C}$ is at least $5\sim10$ times higher than the switching frequency ($> 200kHz$ filter for a $20kHz$ PWM).

## 3. Shunt Topologies 

### A. Three-Shunt Phase Sensing
- Directly monitors U, V, and W phase leg currents. 
- **Advantage**: Always captures current accurately as long as the bottom FET is conducting.
- **Software Action**: You only need 2 ADC readings (by Kirchhoff's $I_u + I_v + I_w = 0$), but if your PWM duty is close to 100%, the bottom FET conduction time shrinks to zero. Software must clamp Maximum Duty (e.g., to 95%) or fallback to reconstructing the 3rd phase via Kirchhoff's law dynamically depending on the active sector.

### B. Single-Shunt Sensing (DC-Link)
- One shunt on the main ground return of the inverter.
- **Advantage**: Cheap, forces all currents through one node.
- **Pain Point**: Recreating the 3 phases mathematically requires parsing the SVPWM sectors into 2 active vectors and sampling TWICE per PWM period in tiny windows between state changes.
- **Asymmetric PWM Constraint**: If the active state is too short (e.g. $T_1 < 2\mu s$), the Current OPAMP cannot settle, and the ADC cannot convert. The software MUST artificially stretch the PWM pulse (Asymmetrical injection) around sector boundaries just to buy enough time to sample. **Do not use 1-shunt unless cost is the absolute limiting factor; it triples the software complexity.**

### C. Inline Sensing (Phase Wire)
- Inductive or Hall-effect sensors (e.g., Allegro ACS) placed right on the phase output wire.
- **Advantage**: Zero blind spots. Independent of PWM state.
- **Pain Point**: Expensive. Highly susceptible to common-mode voltage transients (dv/dt jumping from 0 to bus voltage). Wait until the ringing stops before triggering the ADC.
