# Micromouse Motor-Control — Doc Index

> Engineering decisions documented. Future-you can understand *why* code is way it is.

---

## Docs

| File | Covers |
|---|---|
| `01_ARCHITECTURE.md` | File layout, class responsibilities |
| `02_MOTOR_DRIVER.md` | `MicromouseMotor.h` — PWM config, decay modes, MIN_POWER |
| `03_ENCODER.md` | `MicromouseEncoder.h` — ISR design, direction, `IRAM_ATTR` |
| `04_IMU.md` | `MicromouseIMU.h` — I2C wake, calibration, integration, noise floor |
| `05_PID_LOOP.md` | PID gains, target speed, integral clamp, dt fix, Serial Plotter |

---

## Hardware reference (keep in sync with `PinConfig.h`)

| Subsystem | Part | Key spec |
|---|---|---|
| MCU | ESP32-S3 | Xtensa LX7 dual-core, 3.3V GPIO, hw `ledc` timers |
| Motor driver | DRV8833 | Dual H-bridge, 10V max, ~0.6V FET drop, SLEEP active-LOW |
| Motors | N20 brushed DC, 6V, 1:30 gear, 500RPM no-load | ~8.3 RPS wheel |
| Encoders | Single-channel, 7 pulses/rev (motor shaft) | 7×30 = 210 ticks/wheel-rev |
| IMU | MPU-6500 (I2C 0x68) | ±250°/s gyro, 131 LSB/°/s at FS_SEL=0 |

---

## First upload checklist

- [ ] `DRV_SLEEP_PIN` HIGH in `setup()` — driver won't move without this
- [ ] Battery connected (not USB-only)
- [ ] `MIN_POWER` tuned for motor (start 150)
- [ ] `TICKS_PER_REV` matches encoder mode (210 single / 840 quadrature)
- [ ] Serial Monitor at 115200 baud
