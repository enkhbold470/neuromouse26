# 01 — Architecture

## Overview

The project is split into four `.h` class files plus one `.ino` entry point.
This was a deliberate structural choice, not the way the original AI-generated
code was delivered.

---

## Why split into separate `.h` files instead of one big `.ino`?

The original vibe-coded output dumped everything into `motor-control-test.ino`.
This was changed for three concrete reasons:

1. **Testability** — Each class can be stubbed or swapped independently.
   When you move to a two-motor setup you replace one instantiation line,
   not a 200-line file.

2. **Readability** — The `.ino` entry point should read like a state machine
   (setup → run loop), not like a hardware driver. Mixing PWM timer setup
   with PID math in one file is how bugs hide.

3. **Reuse across sketches** — `MicromouseMotor.h` and `MicromouseEncoder.h`
   can be `#include`d into a maze-solving sketch unchanged.

---

## File responsibilities

### `PinConfig.h`
- Central pin number registry.
- **Every** GPIO number lives here and nowhere else.
- Also holds physics constants (`WHEEL_DIAMETER`, `TICKS_PER_REV`) because
  they are properties of the hardware, not the algorithm.
- Uses `#define` rather than `const int` because these values are used in
  `attachInterrupt()` and `ledcAttach()` which are preprocessor-friendly
  contexts and because `#define` has zero flash/RAM cost.

### `MicromouseMotor.h`
- Owns everything about driving current through one H-bridge channel.
- Knows nothing about encoders, PID, or target speeds.
- Its public API surface is intentionally tiny: `begin()`, `drive(int)`,
  `brake()`, `coast()`.

### `MicromouseEncoder.h`
- Owns tick counting for one encoder channel.
- Owns the ISR logic (`handleInterrupt()`).
- Does **not** compute speed — that is the PID loop's job.
  Speed requires a time measurement, which belongs in the control loop,
  not inside a hardware abstraction layer.

### `MicromouseIMU.h`
- Owns raw I2C communication with the MPU-6500.
- Owns bias calibration and Euler integration for yaw.
- Does **not** run its own timer — `update()` must be called from `loop()`.
  This keeps timing deterministic and avoids hidden jitter from a background
  task racing the PID interrupt.

### `motor-control-test.ino`
- Owns PID state variables, gains, and the 50 Hz tick.
- Owns the ISR *wrapper function* (`leftISR`) — see `03_ENCODER.md` for
  why the wrapper must live here, not inside the class.
- Owns Serial telemetry formatting.

---

## What was intentionally NOT added

| Rejected feature | Reason |
|---|---|
| FreeRTOS tasks / `xTaskCreate` | Adds priority inversion risk; 50 Hz polling on one core is sufficient for motor-control-test phase |
| Library dependencies (e.g. `Encoder.h`, `MPU6050.h`) | Avoids version pinning; the hardware is simple enough to drive directly |
| Separate `.cpp` files | Arduino IDE requires the class definition in the header for single-sketch projects; splitting would require a custom build system |
| Right motor instantiation | Explicitly left for next iteration — adding a half-implemented second motor would cause confusion, not help |
