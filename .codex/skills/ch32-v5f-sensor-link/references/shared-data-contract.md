# Shared Data Contract

The shared data structure is a cross-core ABI. Treat it like a protocol.

Rules:

- V3F and V5F copies must match exactly.
- Existing fields should not be reordered.
- Add new fields at the end unless there is a deliberate migration.
- Use fixed-width types.
- Prefer explicit validity flags and counters.
- For high-rate data, add `update_tick` or a sequence field so V3F can detect staleness.
- Document units in comments and task summaries.

Current important fields:

- `gyro_dps[3]`: roll, pitch, yaw angular rate in degrees per second.
- `rc_roll`, `rc_pitch`, `rc_yaw`, `rc_throttle`: stick values, nominally -120 to +120.
- `rc_sw`: 0 wait, 2 fly.
- `rc_meg`: 0 drop, 1 grab.
- `rc_flags`: spare flags, currently used by V3F KEY4 ramp test.
- `rc_link_ok`: 1 valid link, 0 timeout or invalid.
