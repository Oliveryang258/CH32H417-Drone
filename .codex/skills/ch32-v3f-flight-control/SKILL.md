---
name: ch32-v3f-flight-control
description: Rules and workflow for modifying CH32H417 V3F flight-control code in V3F/User, especially main.c, PWM, PID, mixer, arming, disarming, failsafe, VOFA commands, V307 communication, and safety-critical motor output behavior.
---

# CH32 V3F Flight Control

Use this skill whenever editing `EXAM/GPIO/GPIO_Toggle/V3F/User`.

## Required Context

Before editing V3F flight code, read the relevant files:

- `main.c` for arming, PID loop, mixer, VOFA commands, failsafe, V307 tag handling.
- `bsp_pwm.c` and `bsp_pwm.h` for ESC output semantics.
- `bsp_pid.c` and `bsp_pid.h` for PID limits and reset behavior.
- `bsp_comunicate.c` and `bsp_comunicate.h` for USART1 V307 bytes.
- `shared_data.h` for cross-core fields written by V5F and read by V3F.

## Safety Invariants

- Emergency stop must call `PWM_Lock()`.
- Any forced disarm must set the armed state false, reset PID controllers, clear PID outputs, and clear transient test overrides.
- Disarm paths must not be delayed by throttle ramp or motor slew limits.
- `PWM_SAFE_MAX_US` is a hard per-motor limit. Do not increase it without explicit user approval.
- `THR_MAX_US` is the pilot/override throttle target limit. Do not confuse it with the per-motor hard limit.
- Test modes (`tm`, `tr`, `ts`, `tj`, `gr`) must be cleared when locking or forced-disarming.
- Do not change motor mix signs unless the task is explicitly mixer calibration or motor direction correction.

## V307 Tag Handling

Known one-byte tags on V307-to-V3F USART1:

- `0xAA`: camera on
- `0xAB`: camera off
- `0xBB x_hi x_lo y_hi y_lo`: target coordinates
- `0xBC`: target lost
- `0xCC`: battery low; alarm holds for 500 ms after last tag
- `0xDD`: overcurrent; should force disarm on V3F

Use a state machine when parsing `0xBB` payloads so payload bytes are not mistaken for alarm or failsafe tags.

## Change Workflow

1. Identify whether the task affects motor output, arming, shared data, protocol parsing, or tuning commands.
2. Make the smallest code change that preserves existing test modes and safety behavior.
3. If adding a new forced disarm reason, reuse the same cleanup pattern as gyro overspeed or lock handling.
4. Run static searches for the affected symbols with `rg`.
5. Attempt the V3F build if the toolchain is available.
6. Report hardware validation steps separately from software completion.

## Review Checklist

- Does every disarm path actually stop PWM immediately?
- Can a stale test override survive lock/disarm?
- Can a malformed V307 frame trigger a false alarm or hide a real alarm?
- Does the code preserve RC lost behavior through `rc_link_ok`?
- Did the change avoid generated files under `obj/` and IDE metadata under `.mrs/`?

## References

- Read `references/v3f-safety-rules.md` before changing arming, disarming, PWM, or safety behavior.
- Read `references/mixer.md` before changing motor ordering or mix signs.
