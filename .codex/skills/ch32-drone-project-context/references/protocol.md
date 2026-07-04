# Protocol Notes

V3F and V5F share `SharedSensorData_t` through fixed SRAM at `0x20140000`.

Rules:

- Keep the V3F and V5F copies of `shared_data.h` synchronized.
- Do not reorder fields casually. Add fields only when both cores are updated.
- Prefer explicit validity fields, counters, sequence numbers, or timestamps for new sensor data.
- V5F writes sensor and RC data; V3F owns flight decisions and motor output.
- V5F should not directly decide PWM or flight arming state.

V307-to-V3F USART1 tag rules:

- Preserve the parser state machine.
- Treat `0xCC` as a hold-style low-voltage alarm.
- Treat `0xDD` as an immediate safety event on V3F.
- Extend vision messages with sequence/confidence/drop fields only after documenting both ends.
