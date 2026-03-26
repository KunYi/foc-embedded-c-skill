# Thermal System Modeling and Derating (Reference)

## Overview

A product-grade drive should not treat thermal protection as a single scalar threshold. Continuous current capability, overload duration, and derating behavior depend on where heat is generated, how it moves, and which temperatures are actually measured.

Use this reference when the product must survive sustained load, repeated overloads, enclosure constraints, or varying ambient conditions.

## 1. Thermal Nodes That Matter

Typical motor-drive products have several different thermal nodes:
- motor winding
- stator or housing
- power module or MOSFET case
- heatsink or baseplate
- local PCB hot spots
- ambient air or coolant reference

Do not assume a sensor on one node accurately represents the others.

## 2. Sensor Placement Validity

A temperature sensor is useful only if its physical location is understood.

- A heatsink NTC may lag semiconductor junction stress by a large time constant.
- A PCB temperature channel may say little about winding temperature.
- A motor-frame sensor may miss hot winding rise during low-speed high-current operation.
- MCU internal temperature is not a substitute for module or winding temperature.

Every protection claim should state which thermal node is being observed and which hotter node is being inferred.

## 3. Thermal Time-Constant Awareness

Different parts of the system heat and cool at different rates:
- semiconductor junctions respond quickly
- package and module temperatures respond more slowly
- heatsinks are slower still
- winding and housing behavior depend strongly on motor construction and cooling path

This matters because:
- fast faults may need hardware protection or current limiting before a slow thermal sensor reacts
- long overload allowances must reflect the slowest safe thermal path, not just instantaneous current
- recovery and re-enable logic should account for thermal lag

## 4. Derating Strategy

Derating laws should be monotonic, predictable, and tied to real thermal limits.

Common structure:
- hard thermal fault threshold
- warning threshold below fault
- continuous derating region between nominal and faulted operation

The derating input may come from:
- measured motor temperature
- measured module or heatsink temperature
- conservative `I^2t` overload estimator
- a fused supervisor combining measured temperature and overload history

Avoid derating rules that chatter, fight the control loop, or suddenly restore full torque without thermal recovery.

## 5. Model-and-Measurement Fusion

In many real products, the best thermal strategy is not "sensor only" or "model only", but a combination:

- use direct temperature sensors where they truly represent the protected component
- use `I^2t` or other overload estimators to cover fast or unmeasured stress
- choose the more conservative limit when the two disagree
- define explicit fallback behavior if a temperature sensor opens, shorts, or becomes implausible

## 6. Product-Level Constraints

Thermal design should explicitly answer:
- which node limits continuous torque
- which node limits short overload torque
- which node is authoritative for thermal fault
- what ambient range the product is required to survive
- how enclosure, fan, coolant, or mounting orientation changes the calibration
- whether different product variants need different derating tables

Do not assume one current limit or one derating curve is valid across all assemblies and cooling configurations.

## 7. Acceptance Criteria

- **Sensor validity**: Each thermal sensor must map to a documented physical node and have a defined plausibility range.
- **Derating smoothness**: Torque or current limits must reduce smoothly without oscillation or chatter near thresholds.
- **Trigger ordering**: Warning, derate, and fault actions must occur in the intended order during thermal stress tests.
- **Recovery correctness**: Allowed torque should recover only after measured or estimated thermal state has genuinely fallen back inside the documented margin.
- **Variant correctness**: Product variants with different cooling hardware must use the correct calibration set.

## 8. Manual Verification Plan

- Log current, bus voltage, estimated thermal state, measured thermal channels, and torque limit during sustained overload and cooldown.
- Compare sensor response delay against the expected hot component. Verify the protection does not rely on a sensor that reacts too slowly for the protected node.
- Run repeated overload pulses and verify that derating and recovery remain monotonic across cycles.
- Test high ambient and worst-case enclosure conditions, not only open-bench airflow.
- Force thermal-sensor fault conditions and verify that fallback protection remains conservative.

## 9. Guidance for AI Explanations

When AI explains thermal behavior to an engineer, it should say explicitly:
- which thermal node is being discussed
- whether the signal is measured or inferred
- what the dominant time constant likely is
- whether the protection logic is immediate hardware protection, overload estimation, or slow thermal supervision

Avoid speaking about "temperature" as if the whole product has only one physically meaningful value.
