# Measurement and Instrumentation Best Practices (Reference)

## Overview

Many wrong engineering conclusions come from wrong measurements, not wrong control theory. A product-grade workflow should define how to probe the system without corrupting the very behavior being observed.

Use this reference whenever oscilloscope captures, current probes, differential probes, DAC channels, or telemetry are used to justify design decisions.

## 1. Measurement Is Part of the System

Treat instrumentation as part of the experiment:
- probe capacitance can disturb fast nodes
- poor grounding can invent ringing that is not really there
- insufficient bandwidth can hide real transitions
- excessive bandwidth can expose noise that is not relevant to the question

Before trusting a waveform, state what physical question the measurement is supposed to answer.

## 2. Common Measurement Targets

Typical drive-development measurements include:
- phase node or switching node behavior
- current-sense waveforms and settling
- bus ripple and bus transient behavior
- gate-drive timing and dead-time
- protection-trip latency
- observer or state variables exported through DAC or telemetry

Each target has different probe and grounding requirements.

## 3. Probe Selection Matters

Practical rules:
- use a true differential probe or an equivalent safe technique for high-side or floating measurements
- use current probes appropriate to the bandwidth and current range of the question
- do not use long ground leads on fast switching nodes
- do not assume a low-cost passive probe is valid for every power-stage measurement

When measuring phase-node ringing, the measurement setup itself can change the observed waveform.

## 4. Scope Setup Should Match the Question

Examples:
- to verify trigger placement relative to ringing, prioritize timing resolution and local settling behavior
- to verify bus droop or regen rise, prioritize sufficient time window and voltage scaling
- to verify fault latency, capture both the fault indication and the actual gate disable or output collapse
- to compare current ripple against control activity, align scope channels or logs in time rather than viewing unrelated windows

## 5. DAC, Telemetry, and Internal Signals

Exported internal variables are valuable, but they have limits:
- DAC outputs are bandwidth-limited and may distort fast events
- telemetry may be filtered, decimated, or delayed
- software timestamps may not align with analog events unless designed carefully

Treat DAC and telemetry as observability tools, not perfect truth.

## 6. Common Measurement Mistakes

Watch for:
- measuring a switching node with a long probe ground clip
- confusing probe artifact with real ringing
- trusting a current trace without checking probe zero and bandwidth
- comparing a filtered digital signal against an unfiltered analog waveform as if they were the same quantity
- assuming debug outputs are synchronized when they are not

## 7. Acceptance Criteria

- **Question-fit instrumentation**: the chosen instrument and setup are appropriate to the phenomenon being tested.
- **Grounding correctness**: probe grounding does not obviously inject or exaggerate the observed artifact.
- **Timing alignment**: channels being compared are temporally aligned well enough to support the claimed conclusion.
- **Bandwidth honesty**: the measurement bandwidth matches the engineering question and its limitations are acknowledged.
- **Repeatability**: the same setup produces consistent results across repeated measurements.

## 8. Manual Verification Plan

- repeat critical measurements with at least one alternate setup when possible, especially for fast switching-node phenomena
- document probe type, attenuation, grounding method, bandwidth limit, and trigger condition in saved captures
- check current-probe zero and scaling before drawing conclusions from small-current behavior
- when using DAC or telemetry, compare against at least one trusted analog or scope-based reference to understand the observability gap

## 9. Guidance for AI Explanations

When AI tells an engineer to measure something, it should also say:
- what instrument class is appropriate
- what bad measurement setup could give a false conclusion
- what timing relationship matters
- what the measurement can and cannot prove

Avoid saying "scope this node" without explaining how to do it safely and meaningfully.
