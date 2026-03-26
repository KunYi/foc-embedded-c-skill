# Data Logging, Replay, and Diagnostics Workflow (Reference)

## Overview

A product-grade drive should not rely on memory, screenshots, or tribal knowledge to debug intermittent failures. It should define what data is captured, when it is captured, and how the team can replay or interpret that data after the event.

Use this reference when the product needs reproducible fault diagnosis, field-return analysis, or structured debug workflows.

## 1. Logging Should Answer Specific Questions

Useful logs are not just "more variables." They should help answer:
- what failed first
- what the drive was trying to do at the time
- what the plant was actually doing
- whether the failure started in sensing, control, power, communication, or the application layer

## 2. Minimum Useful Signal Set

At minimum, a useful event record usually includes:
- timestamp or monotonic sample index
- operating mode and state-machine state
- command source and commanded target
- conditioned target after ramps/limits
- speed estimate and, when relevant, position or angle error
- `Id`, `Iq`, and bus voltage
- saturation or limit flags
- fault/warning source
- thermal or protection supervisor state

If current reconstruction or observer validity is under suspicion, log the signals that distinguish the paths rather than only the final fused result.

## 3. Snapshot versus Continuous Logging

Use a mix of:
- **continuous low-rate logging** for trend visibility
- **event-triggered snapshots** for fast transients

Typical snapshot design:
- pre-trigger history
- trigger condition
- post-trigger history

This is often more useful than a large continuous stream that misses the important moment or overloads storage.

## 4. Trigger Conditions

Useful triggers include:
- fault assertion
- warning-to-fault escalation
- communication timeout
- observer unlock or handoff failure
- current saturation or repeated modulation limit
- unexpected reset or watchdog event
- excessive vibration/NVH flag in application-specific systems

## 5. Replay Mindset

A good workflow should let the team replay what happened later:
- reconstruct the order of events
- compare command, conditioned target, and measured response
- distinguish symptom from root cause
- compare one field return against known signatures

Replay does not have to mean a full simulator. Even structured trace review is valuable if the data model is stable and complete enough.

## 6. Diagnostic Classification

Logs should help classify failures into at least these buckets:
- sensing path problem
- control instability or saturation problem
- power-stage or bus disturbance
- host/command contract problem
- application/load-induced behavior

The workflow should not force every issue into "FOC unstable" when the evidence points elsewhere.

## 7. Acceptance Criteria

- **Event usefulness**: a captured log is sufficient to identify the likely first failing layer.
- **Trigger correctness**: important failures create snapshots without flooding storage during normal operation.
- **Time ordering**: timestamps or sequence numbers make event order unambiguous.
- **Schema stability**: the same field means the same thing across firmware versions unless the protocol/schema version changes explicitly.
- **Field diagnosability**: service teams can extract and interpret the data without hidden lab-only knowledge.

## 8. Manual Verification Plan

- Force at least one known fault in each major class and confirm the log captures enough context to diagnose it afterward.
- Verify pre-trigger history exists for fast faults where the interesting cause appears before the trip.
- Compare logs from nominal operation, sensing faults, power faults, and application-induced disturbances to ensure the signatures are meaningfully different.
- Review captured data with a second engineer who did not run the test; if they cannot interpret it, the logging contract is still too implicit.

## 9. Guidance for AI Explanations

When AI recommends a debug workflow, it should say:
- what question the proposed log is meant to answer
- why each suggested signal is needed
- what should trigger the snapshot
- how the resulting data would distinguish likely root-cause categories

Avoid telling engineers to "log everything" without a diagnosis workflow.
