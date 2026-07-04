# V3F Safety Rules

Forced disarm cleanup pattern:

1. Call `PWM_Lock()`.
2. Set the local armed flag to `0`.
3. Clear PID output variables used for telemetry.
4. Reset roll, pitch, and yaw PID controllers.
5. Clear transient test overrides:
   - `g_thr_override`
   - `g_test_jitter`
   - `g_test_sweep_peak`
   - `g_test_ramp_active`
   - `g_test_ramp_start_tick`
6. Reset local safety counters if applicable.
7. Print a concise safety reason if serial logging is already used nearby.

Do not route emergency stops through throttle ramping or motor slew logic.

Safety sources currently present or planned:

- RC switch not in Fly or `rc_link_ok == 0`
- Gyro overspeed over 500 dps for 50 ms
- V307 `0xDD` overcurrent
- Low voltage `0xCC` buzzer warning, not necessarily disarm unless user requests it

Before changing safety code, check that single-motor test mode cannot leave a motor active after lock.
