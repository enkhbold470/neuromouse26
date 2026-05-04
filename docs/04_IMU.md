# 04 — MicromouseIMU.h

## What it does

Init MPU-6500 over I2C, calibrate Z-axis bias at startup, integrate Z-axis rate → yaw angle. `getYaw()` = degrees since last `resetYaw()`.

---

## Decision 1 — Z-axis only (yaw)

Robot moves in flat 2D maze — only vertical-axis rotation matters. Full 6-axis read = 8× I2C bytes + unused float math. Add pitch/roll later if tilt correction needed.

---

## Decision 2 — I2C addr 0x68

```cpp
const int MPU_ADDR = 0x68;
```

AD0 pin LOW (tied to GND on PCB) = 0x68. AD0 HIGH = 0x69 (use if two MPU-6500s on same bus).

---

## Decision 3 — Wake: write 0 to PWR_MGMT_1 (reg 0x6B)

```cpp
Wire.write(0x6B);
Wire.write(0);
```

MPU-6500 boots in sleep mode (bit 6 = 1). Writing 0: clears sleep + CYCLE mode → continuous sampling, selects internal 8MHz oscillator.

For better clock stability:
```cpp
Wire.write(0x01); // PLL with X-axis gyro reference
```

---

## Decision 4 — FS_SEL=0 (±250°/s)

Sensitivity: 131 LSB/°/s.

Fast 90° turn at ~60ms = 1500°/s peak → clips at ±250°/s. **Known limitation** — fine for straight-line motor test.

**For turn control, change to ±2000°/s:**
```cpp
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x1B);
Wire.write(0x18); // FS_SEL=3 → ±2000°/s, 16.4 LSB/°/s
Wire.endTransmission();
// Update: return (float)raw / 16.4;
```

---

## Decision 5 — 200-sample calibration, 2ms delay

```cpp
void calibrate() {
    float sum = 0;
    for(int i = 0; i < 200; i++) {
        sum += readRawZ();
        delay(2);
    }
    gyroBiasZ = sum / 200.0;
}
```

MEMS gyros have zero-rate offset (±1–3°/s) → yaw drifts without bias subtraction.

200 samples: noise ÷ √200 = ÷14.1. `delay(2)` ensures fresh sample each read (MPU refreshes at 1kHz, I2C read ~40µs).

Total: 400ms at startup.

---

## Decision 6 — Noise floor 0.05°/s

```cpp
if (abs(rate) > 0.05) currentYaw += rate * dt;
```

Post-calibration residual noise ~±0.03–0.08°/s. Gate at 0.05°/s prevents drift on still robot. Sub-0.05°/s deliberate rotation ignored — acceptable for fast Micromouse.

Future: Kalman or complementary filter fusing gyro + wheel odometry.

---

## Decision 7 — No IMU interrupt (polling only)

`IMU_INT` (GPIO48) defined but unused. Data-ready interrupt = faster data but needs ISR + shared buffer (concurrency risk). `update()` at ~50Hz from `loop()` sufficient — IMU samples at 1kHz internally, reads most recent register value.

---

## `update()` contract

Call every `loop()` iteration, not just inside PID block. `dt` calculated from `micros()` at call time — missing calls = integration gaps = yaw errors.
