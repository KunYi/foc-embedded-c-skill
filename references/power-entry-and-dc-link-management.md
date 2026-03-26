# Power Entry and DC-Link Management (Reference)

## Overview
Many drive failures happen before the first PWM edge. The DC source, precharge path, contactors, bus capacitors, and brown-in/brown-out behavior define whether the inverter ever reaches a safe state to begin control.

Treat the DC link as a managed subsystem with a lifecycle: unpowered, precharging, ready, enabled, faulted, discharging, and re-armable. Firmware should not assume `Vbus present` means `safe to switch`.

## 1. Power-Entry Questions

Before enabling PWM, define:
- source type: battery, rectified mains, front-end converter, bench supply
- precharge method: resistor, NTC, active precharge, none
- bus connect/disconnect element: relay, contactor, solid-state switch, always connected
- bus-ready criteria: minimum voltage, maximum imbalance, time since precharge start, fault-free status
- discharge strategy after shutdown: bleed resistor, active discharge, passive decay only

## 2. Inrush and Precharge

Charging a large DC-link capacitor directly can damage rectifiers, connectors, relays, or supplies through inrush.

Recommended firmware-visible states:
- `power_off`
- `precharge_active`
- `bus_ready`
- `main_path_closed`
- `bus_fault`

Rules:
- do not enable PWM before precharge is complete and the main path is confirmed
- verify that bus rise rate and final bus voltage match the expected precharge profile
- if bus voltage stalls, rises too slowly, or overshoots unexpectedly, treat it as a power-entry fault rather than trying to start the motor anyway

## 3. DC-Link Stress and Lifetime

The DC link is not just a voltage node; it is an energy-storage and ripple-handling subsystem.

Check:
- capacitor ripple current
- ESR heating
- voltage margin versus worst-case bus surge
- lifetime implications of ambient and ripple temperature
- repeated regen/braking stress on capacitor temperature and voltage headroom

Firmware implications:
- warning thresholds should leave margin below absolute capacitor and power-stage limits
- repeated braking or load cycling may need derating if bus temperature or ripple stress becomes excessive

## 4. Brown-In, Brown-Out, and Supply Collapse

Not every failure is a hard disconnect. Real products see slow bus ramps, sagging supplies, connector bounce, and contactor chatter.

Rules:
- define minimum bus voltage required for valid PWM operation
- if gate driver UVLO or bus sag invalidates output control, disable torque production deterministically
- do not re-arm automatically unless the bus and contactor state satisfy the documented restart conditions
- separate `bus not ready` from `bus faulted`; operators and host software need to know the difference

## 5. Contactor / Relay / Main-Path Sequencing

If the system uses a contactor or relay:
- define when it may close
- define when PWM may begin relative to contact closure
- define when it must open on fault
- define whether a faulted bus must first discharge before re-close

Do not let firmware toggle power-path hardware without a documented sequencing policy.

## 6. Acceptance Criteria

- **Precharge profile**: measured bus rise follows the expected envelope without damaging inrush or relay chatter.
- **Bus-ready logic**: PWM stays inhibited until the documented bus-ready conditions are met.
- **Brown-out response**: torque production is removed deterministically before gate-driver or control validity is lost.
- **Restart policy**: after a bus fault or collapse, the system only re-arms under the documented reset/precharge conditions.
- **DC-link margin**: worst-case bus surge, braking events, and supply transients remain below component and warning/fault thresholds with margin.

## 7. Manual Verification Plan

- Scope source voltage, precharge node, DC bus, contactor/relay command, and `PWM enable` status on the same timeline.
- Measure inrush and bus rise during cold start and hot restart.
- Force brown-out and bus sag conditions and verify the firmware exits torque production cleanly.
- Repeat tests with the highest expected bus voltage, lowest source impedance, and representative bus capacitance.
