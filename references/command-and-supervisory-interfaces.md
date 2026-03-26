# Command and Supervisory Interfaces (Reference)

## Overview
Real products rarely drive a motor from fixed constants alone. A host controller, remote user interface, RC PWM input, or field bus usually commands torque, speed, position, damping, or follow behavior.

Treat this layer as a **supervisory control plane**, not as part of the hard real-time current loop. The command path may run over UART, CAN, PWM input capture, or a higher-level host, but it must never destabilize the motor loop through blocking calls, unbounded parsing latency, or target steps that exceed the plant's safe acceleration and current limits.

## 1. Architectural Separation

Use a layered structure:

- **Communication layer**: UART, CAN, PWM input capture, analog input, etc.
- **Command decoding layer**: frame parsing, CRC/checksum, unit conversion, plausibility checks.
- **Mode manager**: selects torque/current, speed, position, damping/impedance, or follow mode.
- **Trajectory / target conditioning**: slew-rate limiting, jerk limiting, soft limits, timeout fallback.
- **FOC real-time layer**: consumes already-sanitized targets and runs the cascaded control loops.

Hard rule: communication ISR or DMA callbacks may push decoded data into a mailbox or shadow target structure, but must not directly execute the current loop or overwrite live ISR state unsafely.

## 2. Common Command Sources

### A. UART Host Commands
- Good for debug consoles, upper-computer tuning tools, product provisioning, and moderate-rate target updates.
- Risks: framing loss, stale packets, blocking `printf`, and command bursts that step targets faster than the mechanics can follow.
- Recommended use: parse frames in the background, update a shadow command object, then atomically hand the latest valid target to the mode manager.

### B. CAN / CANopen / Proprietary CAN
- Good for distributed motion systems, multi-axis coordination, and robust product communication.
- Risks: bus-off, stale heartbeats, arbitration delays, and silently repeated last commands if timeout handling is weak.
- Recommended use: require heartbeat or command freshness checks and define the fallback behavior explicitly on communication loss.

### C. PWM Input Command
- Common in ESC-style products, RC interfaces, and compatibility control paths.
- Risks: input jitter, pulse-width dropout, ground offset between source and target, and invalid pulse ranges during cable noise or reboot.
- Recommended use: capture with timer input-capture, validate pulse width and update period, low-pass filter the decoded command only as much as the application can tolerate, and enforce a timeout-to-safe-state rule.

### D. Analog Input / Throttle
- Good for simple knobs, pedal/throttle inputs, or legacy compatibility.
- Risks: ADC noise, broken sensor wires, offset drift, ground mismatch, and ambiguity between "zero command" and "sensor fault".
- Recommended use: use plausibility windows, startup zero-calibration only when physically safe, and separate faulted-input handling from legitimate zero demand.

## 3. Commanded Modes

### Unit Contract Before Mode Design
Before defining any control mode, the product must explicitly fix the command units and reference frames. Do not let the host and firmware guess.

Recommended command contract fields:
- command value
- command unit
- reference frame (`mechanical` or `electrical`)
- source timestamp or freshness age
- mode owner / source ID when multiple controllers exist

Examples:
- torque or current command: `Iq_ref` in amperes, or torque in N*m with a clearly defined motor torque constant and any temperature- or saturation-dependent caveats
- speed command: `rad/s` or `RPM`, explicitly marked as mechanical or electrical
- position command: radians, degrees, encoder counts, or turns, with wrap rules and zero reference defined
- follow command: explicitly say whether the follower tracks leader mechanical angle, electrical angle, speed, or torque

### Torque / Current Mode
- The host requests torque-producing current directly, usually through `Iq_ref`.
- Use when the upstream controller already closes the speed or position loop.
- Constraint: the software must still enforce current magnitude, thermal derating, speed-dependent voltage limits, and fault-state overrides.
- Unit rule: if the host sends torque rather than current, define whether the firmware converts torque to `Iq_ref` using a fixed `Kt`, a calibrated table, or a temperature-/field-weakening-aware model. Do not silently treat N*m as amperes.

### Speed Mode
- The host commands target speed; the drive closes the speed loop and outputs `Iq_ref`.
- Use when the mechanics are simple enough that local speed control is meaningful.
- Constraint: speed targets must be ramp-limited so communication jitter or packet loss does not become torque shock.

### Position Mode
- The host commands position or trajectory waypoints.
- Use only when the sensor quality, mechanical backlash, and update path support stable position closure.
- Constraint: position commands need target wrapping rules, mechanical end-stop handling, and a clear ownership model between host trajectory generation and local interpolation.
- Unit rule: define whether commanded position is mechanical shaft angle, electrical angle, post-gearbox angle, or multi-turn actuator position. These are not interchangeable.

### Damping / Impedance Mode
- The drive behaves like a virtual spring-damper or velocity-proportional brake.
- Typical law: `Iq_ref = K_spring * (theta_target - theta_meas) - K_damp * omega_meas`, with current limiting and passivity-minded bounds.
- Constraint: do not expose unbounded virtual stiffness or damping gains without checking current saturation, sensor quantization noise, and transport delay.
- Passivity rule: if the mode is marketed as damping, brake, compliant follow, or impedance control, verify that the net behavior is dissipative over the intended operating range instead of injecting energy because of delay, filtering, or sign mistakes.

### Follow Mode
- The drive follows another axis, host-streamed trajectory, or measured external angle.
- Constraint: define whether follow mode means position follow, speed follow, torque assist, or impedance tracking. These are not interchangeable.
- Reference-frame rule: define explicitly whether follow mode tracks leader mechanical position, electrical position, post-gear ratio position, or a transformed application coordinate.
- The follower must define what happens when updates pause: hold-last, ramp-to-zero, freeze trajectory, or fault.

## 4. Physical and Product Constraints

Prefer these constraints over canned middleware examples:

- **Command timeout safety**: The drive must define what happens when host commands stop arriving. Holding the last torque forever is often unsafe.
- **Rate limiting**: A host may command mathematically valid but physically impossible target steps. Apply acceleration, jerk, current, and position limits before they reach the inner loop.
- **Transport jitter isolation**: Bus jitter and packet burstiness must not directly modulate current-loop timing or torque output.
- **Unit consistency**: Host units must be explicit: electrical angle vs mechanical angle, RPM vs rad/s, current vs torque, encoder counts vs radians.
- **Mode semantics**: A mode name like `follow` or `damping` is insufficient by itself. The product contract must define the physical quantity being controlled, the reference frame, and the allowed operating range.
- **Authority ownership**: Only one layer should own each target at a time. If the host sends position while local firmware also runs a speed ramp, define which one wins.
- **Communication fault containment**: UART framing errors, CAN bus-off, PWM signal loss, or analog input faults must map to a defined degraded mode or safe stop.

## 5. Recommended Software Pattern

Use double-buffered or mailbox-style command handoff:

1. Decode communication data into a non-real-time command structure.
2. Validate CRC/range/freshness and convert into engineering units.
3. Pass the result to a mode manager running at a slower supervisory rate.
4. Condition the target through ramp, filter, or trajectory logic.
5. Hand the final target to the speed/position/current loop at a deterministic boundary.

Recommended rates:
- Current loop: PWM/ISR rate
- Observer and modulation: current-loop rate or a deterministic submultiple
- Speed loop: typically slower than current loop
- Position / mode manager / command decoding: slower still, unless the application explicitly requires high-rate synchronized motion

## 6. Validation and Acceptance Criteria

Do not stop at "packets are received." Validate that the motor behavior matches the intended mode.

- **Command freshness**: Drop or flag stale commands. Acceptance criterion: the drive enters the documented fallback behavior within the defined timeout after communication loss.
- **Target ramping**: Step the host command and verify the internal target ramps according to the configured acceleration/jerk limits rather than instantaneously jumping.
- **Mode transitions**: Switch between torque, speed, and position modes and verify bumpless transfer or a documented transition policy.
- **Unit-contract validation**: Deliberately send commands in each supported unit and confirm the drive interprets them in the documented frame and scaling. Mechanical and electrical quantities must not be silently mixed.
- **PWM input validity**: Sweep input pulse width and update frequency. Acceptance criterion: invalid pulse widths, missing pulses, or excessive jitter are rejected or safely degraded rather than interpreted as valid throttle.
- **CAN/UART robustness**: Inject checksum errors, stale frames, missing heartbeats, or burst traffic. Acceptance criterion: the motor output remains bounded and the mode manager behaves deterministically.
- **Damping / impedance stability**: With the rotor perturbed by hand or by a fixture, verify the commanded current remains bounded and the system feels dissipative rather than self-exciting.
- **Follow mode latency**: Log leader and follower position/speed together. Acceptance criterion: tracking lag stays within the application's defined bound without oscillation or runaway on update dropouts.
- **Passivity / damping validation**: Perturb the mechanism across low and moderate speeds. Acceptance criterion: damping mode reduces motion energy over time and does not amplify oscillation because of delay, sign inversion, or noisy velocity estimation.

## 7. Manual Verification Plan

- **Telemetry to log**: commanded target, conditioned target, active mode, command timestamp age, timeout flag, `Iq_ref`, measured speed, measured position, and fault state.
- **Unit/context metadata**: log or expose the command unit, reference frame, active mode semantics, and any gear or pole-pair mapping needed to interpret the target correctly.
- **Scope or logic-analyzer probes**: UART RX/TX timing, CAN frame cadence, PWM input capture timing, mode-transition GPIO strobes, and DAC-exported internal targets when available.
- **Pass criteria examples**:
  - Communication loss forces the documented fallback within the specified timeout.
  - A full-scale command step does not produce current or speed overshoot beyond the product limits.
  - Switching control modes does not create an uncontrolled torque spike.
  - PWM input jitter or invalid ranges do not cause unintended motion.
- **Failure clues**:
  - Torque jumps exactly at packet boundaries.
  - Position mode oscillates only when host update rate changes.
  - Damping mode becomes noisy near zero speed because velocity estimation or filtering is inadequate.
  - Follow mode runs away when the upstream source pauses briefly.
