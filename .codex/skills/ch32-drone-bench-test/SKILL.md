---
name: ch32-drone-bench-test
description: Human-led bench-test workflow for the CH32H417 drone project, including CH32V203 ESC flashing checks, no-prop motor tests, single-motor PWM tests, current/voltage observation, F330 assembly checks, and preflight validation.
---

# CH32 Drone Bench Test

Use this skill to prepare test plans and logs. Do not execute hardware actions automatically.

## Core Rule

AI prepares checklists, expected observations, and log templates. A human performs flashing, power connection, motor spin, prop installation, and prop-on tests.

## Recommended Sequence

1. Visual inspection.
2. ESC firmware flashing verification.
3. No-prop ESC arming test.
4. No-prop single-motor PWM test.
5. No-prop four-motor idle test.
6. Airframe assembly check.
7. Tethered or restrained low-throttle test.
8. Prop-on test only after all previous checks pass.

## Test Log Fields

- Date/time.
- Firmware version or commit.
- Battery voltage.
- Current draw.
- PWM command.
- Motor index.
- Temperature estimate.
- Sound/vibration notes.
- Pass/fail.
- Next action.

## Stop Conditions

- Unexpected motor start.
- Motor desync, stutter, or repeated restart.
- ESC overheating.
- Current spike beyond expected bench range.
- Battery sag, connector heating, smoke, smell, or reset.
- IMU or controller reset during motor operation.

## References

- Read `references/esc-bench-checklist.md` before ESC or motor bench tests.
