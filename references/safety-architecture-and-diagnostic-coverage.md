# Safety Architecture and Diagnostic Coverage (Reference)

## Overview
Safe motor-control products separate three things clearly:
- nominal control behavior
- diagnostic monitoring behavior
- safety action behavior

A strong control loop is not the same as a safe architecture. This reference defines the boundaries and expectations that keep those concerns from being mixed casually.

## 1. Safety Boundary Questions

Before describing any safety feature, define:
- what hazards the firmware is responsible for mitigating
- what hazards are handled only by hardware
- what hazards remain outside the product safety boundary
- what operator or system action is required after a trip

Do not let the AI imply that all dangerous failures are solved in software.

## 2. Fault Layering

Typical layers:

- **Layer 1: hardware-fast containment**
  - comparator trip
  - gate-driver desat/UVLO
  - break input / STO-equivalent path
- **Layer 2: fast firmware diagnostics**
  - ADC plausibility
  - speed plausibility
  - sensor disagreement
  - bus warning / thermal warning
- **Layer 3: slow supervisory diagnostics**
  - timeout supervision
  - thermal accumulation
  - calibration validity
  - service counters and warning persistence

Each layer should have a defined response time and authority.

## 3. Latched vs Recoverable Faults

Do not treat every fault equally.

Recommended distinction:
- **Latched faults**: require operator or host acknowledgement before re-enable
- **Recoverable faults**: may clear automatically after conditions return to normal
- **Derating states**: continue operating in a reduced-capability mode instead of tripping immediately

Examples:
- overcurrent hardware trip: usually latched
- communication timeout in a noncritical mode: possibly recoverable
- thermal warning: often derating before hard trip

## 4. Watchdogs and Monitoring

Define watchdog policy explicitly:
- current-loop watchdog or ISR heartbeat
- background/supervisory watchdog
- external watchdog if present

Diagnostic monitors should also have plausibility rules:
- sensor value within expected range
- speed sign and direction consistency
- commanded versus measured mismatch windows
- estimator validity versus operating region

## 5. Diagnostic Coverage Expectations

Even without claiming a formal safety standard, document the intended coverage:
- which single-point failures are expected to be detected
- which require redundant sensing or cross-checks
- which are only mitigated by hardware shutdown
- which are logged but not actively controlled

This prevents AI or engineers from overstating the safety integrity of a design.

## 6. Acceptance Criteria

- **Fault classification**: each major fault has a documented category: latched, recoverable, or derating.
- **Watchdog behavior**: loss of expected control-loop or supervisory execution leads to the documented safe response.
- **Plausibility detection**: injected sensor disagreement or impossible state combinations are detected within the expected window.
- **Boundary clarity**: host documentation and firmware behavior agree on which faults inhibit torque, which permit retry, and which require manual intervention.

## 7. Manual Verification Plan

- Inject representative sensor, communication, and execution failures and confirm the correct layer responds first.
- Verify that latched faults remain latched across repeated retries until the documented clear condition occurs.
- Verify recoverable faults do not silently convert into uncontrolled oscillation or repetitive stress.
- Confirm the host-visible fault and state schema reflects the real safety behavior, not a simplified guess.
