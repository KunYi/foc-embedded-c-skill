# STM32G4 FOC Hardware Specifics (Reference)

## Overview
Do not rely purely on software for FOC timing limits and overcurrent shutdown. The STM32G4 has analog peripherals tailored for motor control.

## 1. Internal OPAMPs
(TODO: Describe mapping PGA (Programmable Gain Amplifier) nodes. Note settling times versus sampling speed constraints.)

## 2. Zero-cycle hardware trip (COMP & HRTIM/TIM1)
(TODO: State rule: Overcurrent MUST be tied to analog Comparators (COMP) mapped directly to the `BRK` input of the motor Timer, cutting PWMs with zero software intervention.)

## 3. ADC Injection Queue
(TODO: Explain `JSQR` injected sequence triggers synchronized perfectly to Timer Counter valleys/peaks to guarantee average-current sensing.)
