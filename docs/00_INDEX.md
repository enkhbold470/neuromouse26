# Micromouse Motor-Control — Doc Index

> **Historical notes.** Several files in this folder describe an earlier
> ISR-encoder / motor-test stack. For the **current** competition firmware,
> start with [`CLAUDE.md`](../CLAUDE.md), [`include/README.md`](../include/README.md),
> `include/Tuning.h`, and `include/PinConfig.h`.

---

## Docs

| File | Covers | Currency |
|---|---|---|
| `01_ARCHITECTURE.md` | Older file layout / class sketch | Historical |
| `02_MOTOR_DRIVER.md` | DRV8833 PWM notes | Partially stale (PWM is **200 Hz** in production) |
| `03_ENCODER.md` | ISR encoder design | Historical — production uses **PCNT** |
| `04_IMU.md` | Gyro integration notes | Partially stale — production module is `IMU.h` (I2C) |
| `05_PID_LOOP.md` | Early PID experiments | Historical |
| `TEST_*.md` | Bring-up checklists for bench sketches | Useful; verify against `test/` + `platformio.ini` |
| `RESTORE_MAIN.md` | Old restore checklist | Historical |

---

## Hardware reference (keep in sync with live headers)

| Subsystem | Part | Current firmware note |
|---|---|---|
| MCU | ESP32-S3 | Arduino 2.x LEDC via PlatformIO `espressif32@6.5.0` |
| Motor driver | DRV8833 | `MOTOR_SLEEP` GPIO41 HIGH; PWM **200 Hz**, 10-bit |
| Motors | N20 brushed DC, 1:30, ~500 RPM @ 6V | 2S LiPo |
| Encoders | 7 CPR magnetic disk, PCNT 4× | `MicromouseEncoderPCNT.h` |
| IMU | MPU-6500 | **I2C** `0x68`, ±1000 °/s (`GYRO_SCALE=32.8`) |
| IR | SFH4545 + TEFT4300 ×4 | All sensors 90° to target wall |

## First upload checklist (current main firmware)

- [ ] `MOTOR_SLEEP` (GPIO41) driven HIGH in `setup()` — drivers stay off otherwise
- [ ] Battery connected (USB alone is not enough for meaningful motor load)
- [ ] Board: `esp32-s3-devkitc-1` with USB-CDC (`ARDUINO_USB_CDC_ON_BOOT=1`)
- [ ] Serial Monitor at **115200** baud (may need a board reset after connect)
- [ ] `TELEMETRY=false` in `Tuning.h` for competition / timing-sensitive runs
