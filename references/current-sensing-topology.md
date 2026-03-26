# Current Sensing Topologies (Reference)

## Overview
FOC requires precise phase current recreation. The shunt configuration strictly dictates ADC triggering logic and SVPWM minimum pulse widths.

## 1. Three-Shunt Phase Sensing
(TODO: Describe dual/triple ADC simultaneous injection. Detail the ADC setup linked to TIM1 TRGO.)

## 2. Single-Shunt Sensing
(TODO: Highlight the complexity of single-shunt. Address minimum pulse duration (dead-time + ringing + sample time) and asymmetrical PWM injection for boundary sectors.)

## 3. Inline Sensing vs Low-Side
(TODO: Explain OPAMP common-mode transients and when it is safe to sample.)
