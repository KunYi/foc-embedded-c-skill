# Position Sensors Integration (Reference)

## Overview
Mechanical sensors provide zero-speed torque but require filtering, interpolation, or communication latency compensation.

## 1. Incremental Quadrature Encoders (QEP)
(TODO: Describe STM32G4 advanced timer QEP mode, index alignment (Z-phase), and velocity calculation using M/T methods.)

## 2. Hall Effect Sensors
(TODO: Explain the 60-degree sector logic and the absolute necessity of angle interpolation (`speed * delta_T`) to smooth the commutation.)

## 3. Absolute Encoders (SPI / BISS-C)
(TODO: Warn about digital communication latency and the requirement for latency-forwarding compensation based on current RPM.)
