# 01 — Architecture

## Overview

Split into four `.h` class files + one `.ino` entry point. Deliberate structural choice.

---

## Why split into `.h` files?

1. **Testability** — each class stubbed/swapped independently. Two-motor setup = replace one line, not 200.
2. **Readability** — `.ino` reads like state machine. Mixing PWM setup with PID math = bugs hide.
3. **Reuse** — `MicromouseMotor.h` + `MicromouseEncoder.h` drop into maze sketch unchanged.

---

## File responsibilities

### `PinConfig.h`
- Central GPIO registry. Every pin number lives here only.
- Holds physics constants (`WHEEL_DIAMETER`, `TICKS_PER_REV`) — hardware properties, not algorithm.
- Uses `#define` not `const int` — zero flash/RAM cost.

### `MicromouseMotor.h`
- Owns driving current through one H-bridge channel. Knows nothing about encoders/PID.
- API: `begin()`, `drive(int)`, `brake()`, `coast()`.

### `MicromouseEncoder.h`
- Owns tick counting + ISR logic. Does NOT compute speed — that's PID's job.

### `MicromouseIMU.h`
- Owns I2C comms with MPU-6500, bias calibration, yaw integration.
- No internal timer — `update()` called from `loop()`. Keeps timing deterministic.

### `motor-control-test.ino`
- Owns PID state, gains, 50Hz tick, ISR wrapper, Serial telemetry.

---

## Intentionally NOT added

| Rejected | Reason |
|---|---|
| FreeRTOS tasks | Priority inversion risk; 50Hz polling sufficient |
| External libs (Encoder.h, MPU6050.h) | Avoids version pinning; hardware simple enough to drive direct |
| Separate `.cpp` files | Arduino IDE needs class def in header for single-sketch |
| Right motor | Left for next iteration — half-impl causes confusion |
