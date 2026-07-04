# Rate PID Simulation Notes

This note explains how to use `tools/rate_pid_sim.py` while ESC repair is still pending.

## Purpose

The simulator is only for choosing conservative starting gains and checking control limits. It does not replace bench tests, a PID tuning frame, or real flight tuning.

It models one axis of the current V3F rate loop:

- 200 Hz PID period (`5 ms`)
- same derivative filter as `bsp_pid.c`
- PID output limit of `180 us`
- motor slew limit of `12 us / 5 ms`
- simplified motor lag and airframe angular-rate dynamics

## Current Result

Default robust sweep across weak, nominal, and strong plant assumptions suggests the best mathematical candidates around:

```text
kp = 1.15 .. 1.20
ki = 0.00
kd = 0.000 .. 0.005
```

Do not treat this as the first prop-on flight value. The model does not know the real ESC response, prop thrust, frame inertia, vibration, battery sag, or motor desync behavior.

## Recommended First Real-World Values

Use deliberately lower values for the first restrained or PID-frame test:

```text
Conservative:
rp0.45
ri0
rd0
pp0.45
pi0
pd0

Medium, only after conservative response is stable:
rp0.65
ri0
rd0.001
pp0.65
pi0
pd0.001

Do not start with:
kp >= 1.0
kd >= 0.005
ki > 0
```

## How To Run

```powershell
python tools/rate_pid_sim.py
```

Outputs:

- `tools/output/rate_pid_sweep.csv`
- `tools/output/rate_pid_trace_nominal.csv`

These CSV files are ignored by git.

## How To Use With VOFA

1. Keep props off for command-path checks.
2. Start with `I = 0`.
3. Tune Roll and Pitch rate loops first.
4. Increase `P` until response is clear but not oscillatory.
5. Add tiny `D` only if the response overshoots or rings.
6. Add `I` last, only after P/D are stable.
7. Change one parameter at a time and record the VOFA curve.

The simulator is successful if it prevents obviously wild first guesses. Final values must come from the real airframe.
