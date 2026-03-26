# Servo and Actuator Applications (Reference)

## Overview

Servo and actuator products typically care less about "the motor spins" and more about trajectory quality, reversal behavior, disturbance rejection, hold stability, and mechanical interaction with the load.

Use this reference when the product behaves like an actuator, position servo, or motion stage rather than a simple speed-controlled spinner.

## 1. Typical Product Concerns

Servo-like applications often care about:
- position accuracy
- follow-mode quality
- damping or compliant interaction
- reversal shock
- homing and endstop handling
- load compliance, backlash, and resonance

## 2. Control Questions

Important questions include:
- is the primary contract position, speed, force/torque proxy, or follow behavior?
- does the system hold position against disturbance or only track trajectories?
- what should happen at endstops, homing markers, or motion limits?
- how much overshoot or settling time is acceptable?

## 3. Mechanical Interaction Matters

In servo products, the mechanical system often dominates perceived performance:
- backlash can hide or release stored error suddenly
- compliance can turn fast control into oscillation
- aggressive damping or following gains may excite structure
- holding torque may create thermal or acoustic side effects

## 4. Acceptance Criteria

- **Trajectory quality**: motion tracks the documented profile within the product target for error, overshoot, and settling.
- **Reversal quality**: reversals do not create unacceptable shock, chatter, or backlash slam.
- **Hold stability**: position hold or damping mode remains stable without excessive current hunting or heating.
- **Limit handling**: endstop, homing, or limit transitions follow the defined product contract.

## 5. Manual Verification Plan

- test tracking, reversal, hold, and disturbance rejection separately
- log command, conditioned target, measured position/speed, and current during reversals and limit interactions
- test with the real linkage, gearbox, and load, not only a bare rotor
- validate homing and endstop behavior repeatedly, including recovery after interruption or fault

## 6. Guidance for AI Explanations

When AI discusses a servo or actuator system, it should say:
- what motion contract the product is trying to satisfy
- what mechanical effect is likely to dominate performance
- how to verify trajectory quality, reversal quality, and hold behavior separately

Avoid reducing a servo problem to current-loop tuning alone.
