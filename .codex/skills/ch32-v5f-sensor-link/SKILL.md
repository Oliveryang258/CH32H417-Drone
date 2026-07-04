---
name: ch32-v5f-sensor-link
description: Rules and workflow for modifying CH32V307/V5F sensor, NRF RC link, optical flow, TOF, IMU, shared-memory, and telemetry code under V5F/User in the CH32H417 drone project.
---

# CH32 V5F Sensor Link

Use this skill whenever editing `EXAM/GPIO/GPIO_Toggle/V5F/User`.

## Responsibilities

- V5F owns sensor collection, RC packet reception, link health, and shared-memory writes.
- V5F does not own final flight decisions, arming, disarming, mixer, or PWM motor output.
- V3F reads V5F-provided data and decides motor behavior.

## Required Context

Before editing V5F link code, read the relevant files:

- `main.c` for bring-up loop, IMU/LF/TOF/NRF polling, shared-memory writes.
- `shared_data.h` for the cross-core contract.
- `bsp_imu.c/h`, `bsp_lf.c/h`, `bsp_tof.c/h`, `bsp_nrf.c/h` as relevant.
- V3F `shared_data.h` when changing shared fields.

## Shared Data Rules

- Keep V3F and V5F `SharedSensorData_t` layouts synchronized.
- Do not reorder existing fields unless explicitly migrating both cores.
- Add validity, sequence, timestamp, or counter fields for new sensor streams.
- Write simple stable values to shared memory; avoid exposing half-parsed protocol state.
- Treat `rc_link_ok = 0` as the V5F link-failure signal. V3F owns the motor failsafe.

## RC Link Rules

- Preserve magic and checksum checks before writing RC values.
- Keep `RC_LINK_TIMEOUT_MS` conservative unless the user asks to tune it.
- Increment lost/error counters for diagnosis.
- Do not set `rc_link_ok = 1` before a packet passes validation.

## Change Workflow

1. Identify whether the change affects sensor parsing, RC link, shared memory, or ACK payload telemetry.
2. Keep bring-up prints useful but avoid blocking flight-relevant polling paths.
3. If changing shared memory, update both V3F and V5F headers and report the contract change.
4. Run `rg` for changed symbols across both `V3F/User` and `V5F/User`.
5. Attempt the V5F build if the toolchain is available.

## References

- Read `references/shared-data-contract.md` before changing shared-memory fields.
