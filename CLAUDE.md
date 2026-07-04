# CH32H417 Drone Project Rules

Use this file as Claude Code's project entry point. Codex skill drafts live under `.codex/skills/`.

## Project Map

- Root: `C:\Users\32142\Desktop\EVT`
- Main workspace: `EXAM/GPIO/GPIO_Toggle`
- V3F flight controller: `EXAM/GPIO/GPIO_Toggle/V3F/User`
- V5F sensor/link coprocessor: `EXAM/GPIO/GPIO_Toggle/V5F/User`
- ESC firmware: `C:\Users\32142\Desktop\CH32V203C8T6_BLDC_ESC`

## Division Of Labor

- Codex owns local code edits, patches, build attempts, static checks, and git diffs.
- Claude Code owns architecture notes, design review, flight-log interpretation, PID reasoning, and second review.
- Use one owner per task. The reviewer should not edit the owner's files unless the user reassigns ownership.

## Safety Boundaries

- Do not bypass arming, RC link, PWM limits, or disarm logic.
- Do not raise `PWM_SAFE_MAX_US`, `THR_MAX_US`, or PID output limits without explicit user approval.
- Do not automatically flash firmware, spin motors, issue serial motor commands, or perform prop-on tests.
- Treat ESC bench testing and prop-on testing as human-led procedures.

## V3F Rules

- V3F owns arming, disarming, PID, mixer, PWM output, and final failsafe decisions.
- Forced disarm must call `PWM_Lock()`, clear armed state, reset PID, clear PID outputs, and clear transient test modes.
- Preserve state-machine parsing for V307 tags.
- Keep changes small before the 2026-07-09 flight/video milestone.

## V5F Rules

- V5F owns IMU, optical flow, TOF, NRF RC link, and shared-memory updates.
- V5F must not directly command motor PWM.
- Any change to `SharedSensorData_t` must update both V3F and V5F copies of `shared_data.h`.

## Safety Review Rules

- Review arming, RC link, PWM limits, disarm cleanup, sensor validity, and protocol parsing before hardware tests.
- List blocking safety findings first.
- Separate software completion from hardware validation.

## Bench Test Rules

- AI prepares checklists and logs; humans perform flashing, power, motor spin, and prop-on actions.
- Props stay off for ESC flashing, single-motor tests, and first four-motor idle tests.
- Stop on desync, overheating, current spike, reset, smoke, smell, or unexpected motor start.

## Task Header Template

```text
Owner:
Reviewer:
Task:
Scope:
Hardware action:
```

## Example Task Headers

```text
Owner: Codex
Reviewer: Claude Code
Task: Add V3F 0xDD overcurrent disarm.
Scope: EXAM/GPIO/GPIO_Toggle/V3F/User/main.c
Hardware action: none
```

```text
Owner: Claude Code
Reviewer: Codex
Task: Design the next V307-to-V3F vision message format.
Scope: protocol design only
Hardware action: none
```
