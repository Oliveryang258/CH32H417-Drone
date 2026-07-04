# Expanded Safety Checklist

Software:

- Arming requires low throttle and valid RC link.
- Lock/disarm clears test overrides.
- Emergency stop bypasses slew/ramp.
- PWM values remain within documented limits.
- Protocol parsers cannot confuse payload bytes with alarm tags.
- Low-voltage behavior is explicit: alarm only or disarm, as requested.
- Overcurrent behavior is explicit and fail-safe.

Hardware:

- Props off unless the specific step requires props.
- Bench power path can handle expected current.
- ESC signal ground and power ground are correct.
- Motor order and spin direction are verified before flight PID tuning.
- Electromagnet has a proper MOSFET driver and flyback protection.
- IMU is mechanically isolated from high-current wiring and payload switching.

Review response:

- Mark blockers first.
- Then list non-blocking risks.
- Then list required validation steps.
