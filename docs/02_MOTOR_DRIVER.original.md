# 02 — MicromouseMotor.h

## What this class does

Abstracts one DRV8833 H-bridge channel (two PWM pins → one motor direction +
speed). The PID loop in `motor-control-test.ino` calls `drive(int speed)` with
values from −1023 to +1023 and never touches GPIO or ledc directly.

---

## Decision 1 — PWM frequency: 20 kHz

```cpp
const uint32_t PWM_FREQ = 20000;
```

**Why 20 kHz specifically?**

Human hearing range is 20 Hz – 20 kHz. At any PWM frequency below ~18 kHz the
motor coil and driver FETs emit an audible whine. 20 kHz sits just above the
threshold — the motor runs silently.

**Why not higher (e.g. 40 kHz)?**

The DRV8833's FETs have a switching loss that increases with frequency. Above
~30 kHz you start losing meaningful efficiency for marginal gain. 20 kHz is
the industry sweet-spot for small brushed DC motors.

**Why not lower (e.g. 1 kHz)?**

The Arduino default (490 Hz / 980 Hz) causes the motor to hum loudly and
produces jerky motion at low duty cycles because the H-bridge sees discrete
current pulses, not a smooth average.

---

## Decision 2 — PWM resolution: 10-bit (0–1023)

```cpp
const uint8_t PWM_RES = 10;
```

**Why 10-bit, not 8-bit (0–255)?**

More granularity = smoother PID output. With 8-bit you have only 255 steps
across the motor's full speed range. The PID integral can jump by ±1 step and
cause visible speed bumps. 10-bit gives 4× more steps for the same hardware
cost.

**Why not 12-bit (0–4095)?**

The ESP32-S3 ledc timer supports 12-bit, but at 20 kHz:
- `Timer clock = PWM_FREQ × 2^RES`
- At 12-bit: `20000 × 4096 = 81.9 MHz` — exceeds the 80 MHz APB clock
- At 10-bit: `20000 × 1024 = 20.5 MHz` — safely within APB limits

Going to 12-bit at 20 kHz would silently overflow the timer and produce garbage
PWM frequencies.

---

## Decision 3 — `MIN_POWER = 150` (friction feedforward)

```cpp
const int MIN_POWER = 150;
```

**The problem it solves:**

Every brushed DC motor has a *stiction deadband* — a range of PWM values where
the applied voltage is not high enough to overcome bearing friction and DRV8833
FET drop, so the motor does not move at all. The PID integral winds up trying
to push through this deadband, causing:
- Overshoots when the motor finally breaks free
- Oscillation at low speeds
- The motor simply sitting still at low target speeds

**How `map()` removes the deadband:**

```cpp
int pwm = map(speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
```

This re-maps the PID's linear output range [1, 1023] onto the motor's
*effective* range [150, 1023]. The PID never sends a value below 150 when it
intends to move, so the motor always starts.

**Why 150 specifically?**

The DRV8833 drops ~0.6 V per active FET path (two FETs in series = ~1.2 V).
At 6 V supply, net motor voltage = 4.8 V at full PWM.
Minimum voltage to spin this N20 (stiction) ≈ ~1.0–1.5 V.
`1.5 V / 6.0 V × 1023 ≈ 256` absolute minimum, but in practice the FET drop
is voltage-dependent and stiction varies. 150 is a conservative starting point
that was empirically set — **tune this for your specific motor**. If the motor
stutters at low speed, lower it. If it lurches from a stop, raise it.

**Original bug:** the vibe-coded version had `MIN_POWER = 40`. At 40/1023 ×
6 V ≈ 0.23 V — far below any real stiction threshold. The motor did not move.

---

## Decision 4 — Fast decay (coast) vs slow decay (brake) wiring

```
Forward:  IN1 = PWM,  IN2 = 0   → Fast Decay Forward
Reverse:  IN1 = 0,   IN2 = PWM  → Fast Decay Reverse
Brake:    IN1 = HIGH, IN2 = HIGH → Slow Decay (active brake)
Coast:    IN1 = 0,   IN2 = 0    → Fast Decay Zero (freewheeling)
```

**Why fast decay for normal driving?**

Fast decay (one pin PWM, one pin LOW) lets current recirculate through the
body diodes on the off-cycle, which gives the motor a smooth average torque
proportional to duty cycle. This is what the PID equation assumes — a linear
relationship between output and speed.

**Why slow decay for brake?**

Slow decay (both pins HIGH) short-circuits the motor windings. Back-EMF
generates a braking current, stopping the motor rapidly without mechanical
stress. This is used at end-of-move, not mid-PID.

**Original bug:** the vibe-coded version called `brake()` in `coast()` and
had `coast()` in `brake()` — they were swapped. Active braking was being
applied when the PID output was zero, causing the motor to resist any
movement and preventing the PID from ever spinning up.

---

## Decision 5 — Constructor initialization list

```cpp
MicromouseMotor(uint8_t in1, uint8_t in2) : pinIN1(in1), pinIN2(in2) {}
```

**Why an initialization list instead of assignment in body?**

For primitive types like `uint8_t` it makes no functional difference, but
it is idiomatic C++ and signals clearly that these members are set at
construction time and not mutated afterward. It also avoids a double-write
(default-init then assignment) if these were class-type members.

---

## API contract

| Method | Input | Side effect |
|---|---|---|
| `begin()` | none | Attaches ledc to both pins, calls `coast()` |
| `drive(int speed)` | −1023 to +1023 | Sets PWM; 0 calls `coast()` |
| `brake()` | none | Both pins HIGH → slow decay stop |
| `coast()` | none | Both pins LOW → freewheeling |
