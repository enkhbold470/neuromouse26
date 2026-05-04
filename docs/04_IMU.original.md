# 04 — MicromouseIMU.h

## What this class does

Initialises an MPU-6500 gyroscope over I2C, calibrates the Z-axis zero-rate
bias at startup, and integrates the Z-axis rate into a running yaw angle.
`getYaw()` returns degrees rotated since last `resetYaw()`.

---

## Decision 1 — Why only the Z-axis (yaw)?

A Micromouse robot moves in a flat 2D maze. The only rotation that matters for
wall-following and turn correction is rotation about the vertical axis (yaw).
Reading all six axes (3-axis accel + 3-axis gyro) would:
- Triple the I2C transaction time (16 bytes instead of 2)
- Add floating-point processing for unused axes
- Provide no benefit at this stage

X-axis (pitch) and Y-axis (roll) can be added later if you want tilt correction
or odometry fusion, but that is a different module.

---

## Decision 2 — I2C address 0x68 (not 0x69)

```cpp
const int MPU_ADDR = 0x68;
```

The MPU-6500 AD0 pin selects the I2C address:
- AD0 = LOW (default, tied to GND) → address 0x68
- AD0 = HIGH (pulled to VCC) → address 0x69

0x68 is used because the PCB has AD0 tied low, which is the standard
Micromouse PCB default. If you have two MPU-6500s on the same bus, one must
have AD0 pulled high and use 0x69.

---

## Decision 3 — Wake sequence: write 0 to PWR_MGMT_1 (register 0x6B)

```cpp
Wire.write(0x6B); // PWR_MGMT_1
Wire.write(0);    // Wake up (clear sleep bit)
```

The MPU-6500 powers up in **sleep mode** (bit 6 of PWR_MGMT_1 = 1). Writing 0
to the entire register:
- Clears the sleep bit → sensor wakes up
- Clears CYCLE mode → continuous sampling (not low-power periodic)
- Selects internal 8 MHz oscillator as clock source

Writing 0x01 instead (selecting PLL with X-axis gyro reference) gives slightly
better clock stability. For Micromouse use the 8 MHz internal oscillator is
adequate — if you need more timing precision, change to:
```cpp
Wire.write(0x01); // PLL with X-axis gyro reference (more stable)
```

---

## Decision 4 — FS_SEL = 0 (±250 °/s full scale)

The gyro full-scale range was left at the power-on default (FS_SEL register
not written → stays 0 = ±250 °/s).

**Sensitivity:** 131 LSB per °/s (hardcoded in `readRawZ()`)

**Why ±250 °/s and not ±2000 °/s?**

A Micromouse turning 90° in a standard 180 mm cell at competition speed takes
roughly 50–100 ms. Peak angular rate during that turn:
- 90° / 0.06 sec ≈ 1500 °/s for a very aggressive turn

At ±250 °/s, the sensor would clip on fast turns. **This is a known limitation
in the current code.** For motor-control testing (going straight) the robot
does not turn fast, so ±250 °/s is fine.

**When you add turn control, change FS_SEL:**
```cpp
// Register 0x1B = GYRO_CONFIG
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x1B);
Wire.write(0x18); // FS_SEL = 3 → ±2000 °/s, 16.4 LSB per °/s
Wire.endTransmission();
// Also update: return (float)raw / 16.4;
```

---

## Decision 5 — 200-sample calibration with 2 ms delay between samples

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

**Why calibrate at all?**

Every MEMS gyroscope has a zero-rate output offset. Even when the sensor is
perfectly still, it reports a small non-zero angular rate (typically ±1–3 °/s
for MPU-6500). Without subtracting this bias, the yaw angle drifts visibly
within seconds.

**Why 200 samples?**

The MPU-6500 gyro output has ~±0.05 °/s random noise (from the datasheet at
250 °/s FS). Averaging N samples reduces noise by √N:
- 10 samples: noise ÷ 3.2 — barely better
- 100 samples: noise ÷ 10 — decent
- 200 samples: noise ÷ 14.1 — good enough for a straight run

**Why `delay(2)` between samples?**

The MPU-6500 default sample rate with the digital low-pass filter enabled is
1 kHz. At I2C 400 kHz, each 2-byte read takes ~40 µs. Without delay, you'd
sample the same internal register value multiple times before it refreshes.
`delay(2)` = 2 ms ensures each sample is a fresh measurement.

**Total calibration time:** 200 × 2 ms = **400 ms** — this is the pause you
see at startup before the robot reacts.

---

## Decision 6 — Noise floor: ignore rates below 0.05 °/s

```cpp
if (abs(rate) > 0.05) currentYaw += rate * dt;
```

After bias subtraction, remaining noise is typically ±0.03–0.08 °/s. Rates
smaller than 0.05 °/s are classified as noise and discarded, not integrated.
Without this gate, random noise integrates into a slow yaw drift even on a
completely still robot.

**Trade-off:** very slow deliberate rotations below 0.05 °/s are ignored. For
a fast-moving Micromouse this is fine — you never want to detect sub-0.05 °/s
rotation as meaningful.

**Limitation of this approach:** the noise floor is fixed. A proper
implementation would use a Kalman filter or complementary filter to fuse gyro
with wheel odometry. That is a future module.

---

## Decision 7 — No hardware interrupt use for IMU (polling only)

`IMU_INT` (GPIO 48) is defined in `PinConfig.h` but not used. The MPU-6500
data-ready interrupt fires at the sample rate (up to 8 kHz). Reading via
interrupt would give faster data but requires:
- A separate ISR
- A shared buffer between ISR and main loop (concurrency hazard)
- More careful timing design

For the motor-control-test phase, `update()` called in `loop()` at ~50 Hz is
sufficient. The IMU samples at 1 kHz internally; calling `readRawZ()` at 50 Hz
just reads the most recently completed sample from the output registers. There
is no data loss — just less frequent integration updates.

---

## `update()` timing contract

`update()` **must** be called every iteration of `loop()`, not just inside the
PID interval block. This is because `dt` is calculated from `micros()` at the
time of the call. If `update()` is only called at 50 Hz, the `dt` values are
still correct (20 ms each). But if it is called inside the PID block it misses
the continuous integration between PID ticks and yaw errors accumulate.
