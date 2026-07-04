# Mixer Reference

Current intended X layout, viewed from above:

```text
Front
M2 FL CW     M1 FR CCW
M3 RL CCW    M4 RR CW
Rear
```

User-provided current mix definition:

```text
M1(FR) = T - Roll - Pitch + Yaw
M2(FL) = T + Roll - Pitch - Yaw
M3(RL) = T + Roll + Pitch + Yaw
M4(RR) = T - Roll + Pitch - Yaw
```

Before editing mixer signs, compare the code comments and actual expressions. If they disagree, do not "fix" silently; report the mismatch and ask for a motor response test or explicit direction.

Motor-output changes must preserve:

- `mix_clamp()`
- `PWM_SAFE_MAX_US`
- single-motor test override behavior
- motor slew behavior outside single-motor test mode
