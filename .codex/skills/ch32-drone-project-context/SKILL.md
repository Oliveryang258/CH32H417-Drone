---
name: ch32-drone-project-context
description: Project context, roles, safety boundaries, paths, and coordination rules for the CH32H417 competition drone. Use when Codex works on this repository, reviews drone tasks, plans V3F/V5F/ESC changes, coordinates with Claude Code, or needs the project-specific hardware and software map.
---

# CH32 Drone Project Context

Use this skill first for CH32H417 drone work. Keep the flight stack simple, safety-first, and deadline-driven.

## Project Map

- Repository root: `C:\Users\32142\Desktop\EVT`
- Main workspace: `EXAM/GPIO/GPIO_Toggle`
- V3F main flight controller: `EXAM/GPIO/GPIO_Toggle/V3F/User`
- V5F sensor/link coprocessor: `EXAM/GPIO/GPIO_Toggle/V5F/User`
- ESC firmware: `C:\Users\32142\Desktop\CH32V203C8T6_BLDC_ESC`
- Competition targets: flight plus video by 2026-07-09, provincial contest on 2026-07-25.

## Architecture

- Airframe: F330 quad, X mixer.
- Motors: 2212 1400KV x4, 8045 props, 3S 2200mAh.
- V3F/CH32H417: flight PID, arming, disarming, PWM, mixer, failsafe.
- V5F/CH32V307: IMU, optical flow, TOF, NRF RC link, shared memory updates.
- CH32V203 ESCs: BLDC firmware, safety parameters already modified, bench test pending.

## AI Division Of Labor

- Let Codex own code edits that need local inspection, patching, build attempts, static checks, or git diffs.
- Let Claude Code own architecture notes, design alternatives, flight log interpretation, PID tuning reasoning, and second review.
- Do not let both agents edit the same task. Assign one owner and one reviewer.
- For safety-critical changes, have the non-owner review the diff before hardware testing.

## Current Priority Order

1. ESC flashing and bench test.
2. F330 assembly and Roll/Pitch/Yaw PID tuning.
3. V3F overcurrent tag `0xDD` triggers disarm.
4. Vision target coordinate handling from `0xBB`.
5. Electromagnet drop GPIO and release logic.

## Safety Boundaries

- Never bypass arming checks, RC link checks, PWM limits, or disarm paths for convenience.
- Never raise `PWM_SAFE_MAX_US`, `THR_MAX_US`, or PID output limits unless the user explicitly asks and accepts the risk.
- Never perform flashing, serial motor commands, or prop-on testing automatically.
- Treat ESC bench tests and prop-on tests as human-led hardware procedures.
- Prefer small changes before 2026-07-09. Avoid large refactors until basic flight is stable.

## References

- Read `references/protocol.md` before changing V307-to-V3F tags or shared data fields.
- Read `references/workflow.md` before coordinating Codex and Claude Code on a task.
