# DQ Transform & CORDIC (Reference)

## Overview
Clarke, Park, and Inverse Park transforms heavily rely on trigonometric functions (sin/cos).

## 1. Clarke / Park Transformation
(TODO: Outline the mathematical matrix, focusing on amplitude-invariant versus power-invariant logic.)

## 2. STM32G4 CORDIC Integration
(TODO: Explain how to asynchronously trigger the CORDIC peripheral for concurrent `sin(theta)` and `cos(theta)` generation, bypassing `arm_sin_f32()` CPU bottlenecks.)
