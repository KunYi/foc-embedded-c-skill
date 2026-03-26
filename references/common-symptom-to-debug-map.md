# Common Symptom to Debug Map (Reference)

## Overview

Many engineers start from a symptom, not a formula. A useful skill should help translate real product complaints into likely fault domains and the first measurements that separate them.

Use this reference when the observed problem is known, but the root cause is not.

## 1. How To Use This Map

For each symptom:
- identify the most likely fault domains
- measure the smallest set of signals that separates them
- avoid changing multiple subsystems at once
- confirm whether the symptom is electrical, sensing, power-stage, application, or mechanical in origin

This map is for triage, not instant certainty.

## 2. Common Symptoms

### A. Motor twitches or kicks at startup
Likely domains:
- phase order or direction mismatch
- sensor polarity or angle offset issue
- open-loop alignment logic
- wrong current-sign convention

First checks:
- commanded direction versus measured direction
- phase order and angle alignment
- current polarity during first motion

### B. Only certain speeds vibrate or sound bad
Likely domains:
- resonance band
- torque ripple or dead-time artifact
- observer lag or Hall quantization
- application/load coupling

First checks:
- current ripple versus speed
- electrical frequency versus mechanical speed
- vibration or acoustic level versus operating point

### C. Works cold, fails when warm
Likely domains:
- thermal drift in sensing
- protection thresholds too tight
- gate-driver or component drift
- load or process state changing with temperature

First checks:
- current-sense offset/gain drift
- thermal channels and derating state
- fault logs around the failure temperature

### D. Trips overcurrent unexpectedly
Likely domains:
- real current spike
- false trip from noise or threshold placement
- desaturation or gate-drive issue
- current reconstruction error or invalid sample window

First checks:
- comparator threshold and timing
- current waveform at the event
- trigger instant relative to PWM ringing

### E. Observer loses lock or handoff fails
Likely domains:
- insufficient BEMF or wrong handoff criteria
- wrong sign or scaling
- noisy current or voltage estimate path
- transition logic mistake

First checks:
- observer error signal
- estimated speed versus forced/open-loop speed
- transition criteria logs

### F. Deceleration causes bus overvoltage
Likely domains:
- regen path unavailable
- brake chopper missing or undersized
- negative torque limit too aggressive
- bus-energy assumptions wrong

First checks:
- source sink capability
- bus voltage rise rate
- braking mode and torque command

## 3. What Not To Do First

Avoid:
- retuning every PI loop before checking sign, sampling, and phase order
- blaming "FOC instability" before checking current measurement quality
- adding filters before understanding what frequency or artifact is actually present
- assuming a pressure-, flow-, or load-dependent symptom is purely electrical

## 4. Acceptance Criteria

- **Domain narrowing**: the first debug pass reduces the problem to a smaller set of credible fault domains.
- **Measurement relevance**: the chosen measurements actually distinguish those domains.
- **Change discipline**: only one meaningful variable or subsystem is changed at a time during triage.
- **Root-cause honesty**: the team distinguishes symptom suppression from real root-cause identification.

## 5. Guidance for AI Explanations

When AI responds to a symptom-driven question, it should:
- list the most likely fault domains
- say what to measure first
- explain what each measurement would distinguish
- warn what not to change prematurely

Avoid jumping directly to a control-law rewrite when the symptom still has multiple plausible physical causes.
