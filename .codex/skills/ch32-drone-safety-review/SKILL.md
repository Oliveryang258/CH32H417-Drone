---
name: ch32-drone-safety-review
description: Safety review checklist for CH32H417 drone changes involving arming, disarming, PWM, ESCs, motor tests, battery/current alarms, RC link, sensor validity, V307 communication, vision-based control, or payload drop behavior.
---

# CH32 Drone Safety Review

Use this skill before approving safety-critical code changes or hardware test procedures.

## Review Areas

- Arming and lock conditions.
- RC link loss behavior.
- PWM hard limits and throttle limits.
- Forced disarm cleanup.
- Sensor validity and stale-data handling.
- V307 alarm/protocol parsing.
- ESC bench-test safety.
- Electromagnet drop timing and electrical isolation.

## Code Review Checklist

- Does every emergency path call `PWM_Lock()` directly?
- Are PID outputs and integrators reset after forced disarm?
- Are transient test modes cleared after lock/disarm?
- Can malformed serial data produce unsafe motor output?
- Can stale RC or stale sensor data keep the aircraft armed?
- Did the change avoid raising current, PWM, or throttle limits?
- Did the change avoid generated files under `obj/` and IDE metadata under `.mrs/`?

## Hardware Review Checklist

- Props removed for ESC and motor bench tests.
- Airframe fixed before high-throttle tests.
- Battery current and voltage monitored.
- One variable changed per test.
- Test result recorded before changing parameters again.
- Human confirms each flash, spin, or prop-on action.

## Output Format

Lead with blocking safety findings. If no blocker exists, state residual risks and the exact hardware validation needed.

## References

- Read `references/safety-checklist.md` for the expanded checklist.
