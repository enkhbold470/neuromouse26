# Micromouse Motor-Control — Vibe-Coding Documentation Index

> Every engineering decision made during this session is documented here.
> These docs exist so future-you (or a teammate) can understand *why* the code
> is the way it is — not just *what* it does.

---

## Documents in this folder

| File | What it covers |
|---|---|
| `01_ARCHITECTURE.md` | Overall file layout, class responsibilities, and why each class exists |
| `02_MOTOR_DRIVER.md` | `MicromouseMotor.h` — PWM config, decay modes, friction feedforward, MIN_POWER |
| `03_ENCODER.md` | `MicromouseEncoder.h` — ISR design, direction detection, `volatile`, `IRAM_ATTR` |
| `04_IMU.md` | `MicromouseIMU.h` — I2C wake-up sequence, calibration, integration, noise floor |
| `05_PID_LOOP.md` | `motor-control-test.ino` — PID gains, target speed, integral clamp, dt bug fix, Serial Plotter |

---

## Hardware reference (always keep this in sync with `PinConfig.h`)

| Subsystem | Part | Key spec |
|---|---|---|
| MCU | ESP32-S3 | Xtensa LX7 dual-core, 3.3 V GPIO, hardware `ledc` timers |
| Motor driver | DRV8833 | Dual H-bridge, 10 V max, ~0.6 V FET drop per path, SLEEP active-LOW |
| Motors | N20 brushed DC, 6 V, 1:30 gear, 500 RPM no-load | ~8.3 RPS wheel, stall current watch |
| Encoders | Single-channel hall/optical, 7 pulses/rev (motor shaft) | Effective: 7×30 = 210 ticks/wheel-rev |
| IMU | MPU-6500 (I2C 0x68) | ±250 °/s gyro, 131 LSB/°/s at FS_SEL=0 |

---

## Quick-start: first upload checklist

- [ ] `DRV_SLEEP_PIN` driven HIGH in `setup()` — driver won't move without this
- [ ] Battery connected (not USB-only — motor needs real current)
- [ ] `MIN_POWER` in `MicromouseMotor.h` tuned for your specific motor (start 150, lower if stuttering)
- [ ] `TICKS_PER_REV` in `PinConfig.h` matches your encoder mode (210 single / 840 quadrature)
- [ ] Open Serial Monitor at 115200 baud to watch PID telemetry
