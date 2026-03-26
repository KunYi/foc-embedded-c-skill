# NVH and Acoustic Optimization (Reference)

## Overview

A motor drive can be mathematically stable and still be commercially poor if it whistles, chatters, clicks during transitions, or excites mechanical resonance.

Use this reference when the product must manage audible noise, torque ripple, vibration, or customer-perceived smoothness.

## 1. Common Noise Sources

Audible or felt behavior often comes from:
- PWM frequency inside or near the audible band
- dead-time distortion near current zero crossing
- current reconstruction errors near minimum pulse width
- Hall quantization or poor interpolation
- six-step commutation torque ripple
- mode transitions or badly conditioned host targets
- structural resonance in the load, gearbox, or enclosure

## 2. Firmware Levers That Affect NVH

Important control-side contributors include:
- PWM frequency selection
- modulation method (`SVPWM`, `DPWM`, asymmetric PWM, six-step)
- dead-time compensation strategy
- current-loop bandwidth and noise sensitivity
- speed-loop filtering and target slew limiting
- observer noise or phase lag at low speed
- field-weakening and saturation behavior near voltage limits

## 3. Practical Trade-Offs

There is no single "quietest" implementation.

Examples:
- raising PWM frequency may reduce audible whine but increase switching loss and sensing difficulty
- discontinuous PWM may reduce switching loss but create more acoustic asymmetry
- aggressive dead-time compensation may improve THD yet worsen low-current noise if sign estimation is unstable
- stronger filtering may quiet signals but add lag and reduce disturbance rejection

## 4. Product-Level Questions

The design should explicitly define:
- what noise band matters to the customer
- whether efficiency or acoustic comfort is the primary objective
- whether low-speed smoothness matters more than peak efficiency
- whether the drive must avoid exciting a known resonance band
- whether different modes need different acoustic tuning

## 5. Acceptance Criteria

- **Idle noise**: no abnormal tonal whistle, chatter, or current hunting at zero or near-zero torque beyond the documented product expectation.
- **Low-speed smoothness**: torque ripple, clicking, or Hall-sector harshness remain within the product target.
- **Mode-transition smoothness**: startup, braking, follow-mode entry, and fault-recovery transitions do not create unacceptable audible or felt shocks.
- **Resonance avoidance**: the command conditioning and loop tuning do not repeatedly excite known mechanical resonance bands.
- **Thermal/efficiency sanity**: acoustic improvements do not violate thermal or efficiency limits without being explicitly acknowledged.

## 6. Manual Verification Plan

- Record current, speed, and commanded torque during low-speed operation where acoustic problems are often worst.
- Sweep PWM frequency, dead-time compensation, and current-loop filtering while listening and logging for correlation.
- Test the product with the real enclosure, cabling, and mechanical load, not only a free-spinning motor on the bench.
- Check startup, stop, braking, and host-command transients for clicks or step torque.
- If resonance is suspected, correlate vibration or acoustic peaks with speed, electrical frequency, and command events.

## 7. Guidance for AI Explanations

When AI recommends an NVH change, it should explain:
- what physical or control-side source of noise it is targeting
- what trade-off is being made
- how to measure whether the change actually helped
- what failure symptom would indicate the acoustic fix harmed sensing, efficiency, or protection margin

Avoid claiming "higher PWM is better" or "add filtering" without naming the acoustic mechanism and the measured trade-off.
