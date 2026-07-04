# ESC Bench Checklist

Before power:

- Props removed.
- Frame fixed or motor secured.
- Wiring inspected for shorts, reversed polarity, weak solder joints.
- ESC signal wires connected to the intended PWM channels.
- Battery voltage checked.
- Current measurement available if possible.

No-prop sequence:

1. Power controller without arming motors.
2. Confirm no motor twitches unexpectedly.
3. Arm at minimum PWM.
4. Test one motor at a time.
5. Increase PWM in small steps.
6. Stop immediately on desync, heat, abnormal sound, reset, or current spike.

Record:

```text
Motor:
PWM:
Current:
Voltage:
Sound:
Temperature:
Pass/fail:
Notes:
```

Do not install props until motor order, spin direction, and low-throttle stability are confirmed.
