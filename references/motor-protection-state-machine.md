# Motor Protection & State Machine (Reference)

## Overview
FOC software manages complex transitions between shutdown, open-loop alignment, closed-loop run, and fault recovery.

## 1. Alignment & Open-Loop Startup
(TODO: Describe forced-alignment vectors (e.g., Id = rated, Iq = 0) and exactly how many milliseconds to wait before switching to sensorless or checking Z-index.)

## 2. Fault Detection Boundaries
(TODO: Cover DC bus overvoltage during regenerative braking, phase loss detection (asymmetrical phase currents), and Stall detection (Back-EMF collapse or torque saturation).)

## 3. Soft-Recovery and Catch-Spin
(TODO: Document "Flying Start" mechanisms where the observer runs first to find the spinning rotor's angle before enabling PWM.)
