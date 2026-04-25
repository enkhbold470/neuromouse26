# 05 — motor-control-test.ino (PID Loop)

## What this file does

The `.ino` entry point wires all three hardware classes together and runs a
fixed-rate velocity PID loop at 50 Hz on the left motor. Serial telemetry
is printed every 50 ms, compatible with Arduino Serial Plotter.

---

## Decision 1 — DRV8833 SLEEP pin driven HIGH in setup()

```cpp
pinMode(DRV_SLEEP_PIN, OUTPUT);
digitalWrite(DRV_SLEEP_PIN, HIGH);
```

**This is the single most common reason a motor "does not move."**

The DRV8833 has an active-LOW SLEEP input. When SLEEP is LOW (or floating),
the H-bridge output stage is completely disabled — PWM signals on IN1/IN2 are
ignored. The motor draws no current and does not move regardless of what PWM
you send.

The ESP32-S3 GPIO defaults to high-impedance input after reset. If `SLEEP_PIN`
is left unconfigured, it floats. Depending on PCB pull-down resistors this may
latch the driver in sleep mode.

**Fix:** configure the pin as OUTPUT and drive it HIGH before calling
`leftMotor.begin()`.

This was the **first** bug diagnosed — the vibe-coded version never set this
pin.

---

## Decision 2 — PID interval: 20 ms (50 Hz)

```cpp
const unsigned long PID_INTERVAL_US = 20000; // 20 ms → 50 Hz
```

**Why 50 Hz specifically?**

The PID loop rate must be fast enough to react to disturbances before they grow
uncontrollable, but not so fast that noise dominates the derivative term.

At 50 Hz, the loop period is 20 ms. The slowest disturbance for a Micromouse
motor is mechanical load changes (hitting a wall, bumping a wheel), which
happen on a ~100 ms timescale. A 50 Hz loop reacts 5× faster than the
disturbance, which is adequate.

**Why not faster (e.g. 200 Hz)?**

At 200 Hz with a 1750 ticks/sec encoder:
- Ticks per loop = 1750 / 200 = **8.75 ticks per interval**
- One tick = 11.4% of that count — huge quantization error

Derivative term becomes very noisy because `(error_prev - error_new)` is
dominated by ±1 tick quantization rather than real speed changes.

50 Hz gives 1750 / 50 = **35 ticks per interval** at max speed, so ±1 tick
is only 2.9% error — manageable.

---

## Decision 3 — target_speed = 800 ticks/sec ← THIS NEEDS TUNING

```cpp
float target_speed = 800.0; // ticks/sec
```

**⚠️ Critical note discovered while writing this doc:**

The original comment said `~23% of free-run` based on 1750 ticks/sec free-run.
That calculation assumed 210 ticks/rev × 500 RPM motor shaft speed, which is
**wrong** — 500 RPM is the *output shaft* (post-gearbox) speed, not motor shaft.

Correct calculation:
```
Motor free-run:  500 RPM output shaft
                = 500 / 60 rev/sec = 8.33 rev/sec
                = 8.33 × 210 ticks/rev = 1750 ticks/sec  ✓ (this part was right)

But 800 ticks/sec of encoder ticks means:
  800 / 210 = 3.81 rev/sec = 228 RPM output shaft
  228 RPM / 500 RPM = 45.6% of free-run ← not 23%

This is actually fine — 45% of free-run leaves enough PID headroom.
The comment was wrong; the value 800 is usable.
```

**Recommended starting values:**
| Goal | target_speed | % of free-run |
|---|---|---|
| Very slow / safe first test | 200 | 11% |
| Gentle run | 500 | 29% |
| Normal maze speed | 800 | 46% |
| Fast run | 1400 | 80% |

Start at 200 on the bench. If the motor moves and the Serial Plotter shows
`current_speed` tracking `target_speed`, double it and repeat.

---

## Decision 4 — PID gains: Kp=1.5, Ki=0.8, Kd=0.05

```cpp
float Kp = 1.5;
float Ki = 0.8;
float Kd = 0.05;
```

**Original vibe-coded bug:** gains were `Kp=2.5, Ki=5.0, Kd=0.5`.

`Ki=5.0` with a 20 ms loop and errors up to 800 ticks/sec:
- Integral grows at 5.0 × 800 × 0.02 = 80 units per loop
- After 5 loops (100 ms): integral = 400 → Ki × integral = 2000, which
  saturates the output immediately and causes severe windup oscillation

**Why these specific values?**

These are empirically-safe starting points for a first run, derived from the
motor's time constant. A brushed N20 at 6 V has a mechanical time constant of
roughly 50–100 ms. The PID should react on the same order.

**Ziegler-Nichols starting heuristic:**

Since we can't measure the ultimate gain on a bench, a safe start:
- `Kp`: large enough that Kp × max_error ≈ 0.5 × MAX_PWM
  - max_error ≈ 800 (target speed at start, when speed = 0)
  - 1.5 × 800 = 1200 ≈ MAX_PWM → proportional term fills most of output, fine.
- `Ki`: small enough that integral term takes ~1–2 sec to reach saturation
  - At steady state error 50 ticks/sec: 0.8 × 50 × 0.02 = 0.8 units/loop
  - Reaches 1023 in 1023/0.8 = **1278 loops = 25.5 seconds** → safe
- `Kd`: light damping only, `0.05 × (50/0.02) = 125` — less than 15% of output

**Tuning procedure:**
1. Start with `Ki=0, Kd=0`. Raise `Kp` until motor oscillates, then halve it.
2. Add `Ki` slowly (0.1 increments) until steady-state error disappears.
3. Add `Kd` last (0.01 increments) only if there is visible overshoot.

---

## Decision 5 — Integral anti-windup clamp

```cpp
const float INTEGRAL_MAX = 1023.0f / 0.8f; // ≈ 1278

integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
```

**The problem:**

When the motor is stalled or disconnected, the error stays large and the
integral accumulates indefinitely. When the motor finally starts, the integral
term instantly saturates the output, causing a violent lurch (windup).

**The fix:**

Clamp the integral so that `Ki × integral` can never exceed `MAX_PWM` (1023)
on its own:
```
Ki × INTEGRAL_MAX = MAX_PWM
INTEGRAL_MAX = 1023 / Ki = 1023 / 0.8 ≈ 1278
```

This means the integral can contribute at most the full motor range — it can
push the motor to max speed by itself, but can never push it beyond that.

**If you change Ki, recalculate INTEGRAL_MAX:**
```cpp
const float INTEGRAL_MAX = 1023.0f / Ki;
```

---

## Decision 6 — Low-pass filter on speed measurement (α = 0.5)

```cpp
current_speed = (0.5f * current_speed) + (0.5f * raw_speed);
```

This is an exponential moving average (EMA) with smoothing factor α = 0.5.

**Why filter at all?**

The encoder has ±1 tick quantization error. At 50 Hz with ~35 ticks/interval,
that is ±2.9% speed noise. The derivative term amplifies this:
```
d_error / dt = (error_k - error_{k-1}) / dt
```
A ±1-tick error translates to ±1/0.02 = ±50 ticks/sec² derivative noise.
With Kd=0.05 that contributes ±2.5 PWM units per tick — tolerable.
Without the filter, back-to-back ±1 tick swings give ±4 ticks/sec² →
±200 PWM units of derivative chatter at higher Kd values.

**Why α = 0.5?**

α closer to 1.0 = more smoothing = more lag = slower PID response.
α closer to 0.0 = less smoothing = more noise.
0.5 is a balanced starting point — try 0.7 if the derivative is still noisy,
or 0.3 if the response feels sluggish.

---

## Bug fix — `dt` calculation: digit separator on float literal

```cpp
// BROKEN (causes compile error in avr-g++ / xtensa-g++):
float dt = (now - prev_time) / 1_000_000.0f;

// FIXED:
float dt = (float)(now - prev_time) / 1000000.0f;
```

**Why `1_000_000.0f` doesn't compile:**

C++14 introduced digit separators (`'` in standard C++14, `_` is a GCC
extension for integers only). Digit separators are allowed on *integer*
literals, not on *floating-point* literals. `1_000_000.0f` is a float literal
with an underscore — rejected by both avr-g++ and xtensa-g++.

**Why the explicit `(float)` cast:**

`now - prev_time` is `unsigned long - unsigned long` = `unsigned long` (32-bit).
In C++, dividing an `unsigned long` by a `float` promotes the `unsigned long`
to `float` first — that part is fine. But if you wrote:
```cpp
float dt = (now - prev_time) / 1000000; // integer division!
```
You'd get integer division (result 0 for dt < 1 second), then implicitly
convert 0 to float, then divide by zero in the PID — undefined behavior.
The explicit `(float)` cast makes the intent unambiguous and prevents that
mistake if someone removes the `.0f` suffix later.

---

## Decision 7 — Serial telemetry format for Serial Plotter

```cpp
Serial.printf("%.1f\t%.1f\t%.1f\n", target_speed, current_speed, imu.getYaw());
```

Three tab-separated values, one per line:
1. `target_speed` — flat reference line (PID setpoint)
2. `current_speed` — actual measured speed (what PID is controlling)
3. `imu.getYaw()` — yaw angle in degrees (drift indicator)

**Why tab-separated?**

Arduino Serial Plotter parses tab (`\t`) or comma-separated values and plots
each as a separate line on the same graph automatically. This lets you
visually see the PID tracking in real time without any external tools.

**Why 50 ms print rate (20 Hz)?**

The PID runs at 50 Hz. Printing at 50 Hz would produce 50 Serial writes per
second, which at 115200 baud takes:
- Each `printf` ≈ ~30 characters → 30 × 10 bits = 300 bits
- 50 × 300 bits = 15,000 bits/sec — within 115200 baud budget
- But `printf` itself takes ~50–200 µs which adds jitter to the loop

20 Hz print rate gives clean plots with acceptable update frequency and
zero risk of UART blocking the PID interval.

---

## Timing diagram

```
micros() timeline (not to scale):

t=0           t=20ms        t=40ms        t=60ms
|             |             |             |
[PID#1]       [PID#2]       [PID#3]       [PID#4]
   [IMU.update][IMU.update][IMU.update]...  (every loop() iteration)
          [Print#1]              [Print#2]  (every 50ms)

PID interval:   20 ms (50 Hz) — measured with micros(), not millis()
IMU update:     every loop() call, ~every 100–500 µs depending on I2C
Serial print:   every 50 ms (20 Hz) — measured with millis()
```

**Why `micros()` for PID and `millis()` for Serial?**

`micros()` has 1 µs resolution — necessary for accurate `dt` calculation in
the PID (20 ms interval measured to ±1 µs precision).

`millis()` has 1 ms resolution — sufficient for print scheduling, and
`millis()` is slightly cheaper to call than `micros()`. No reason to use
`micros()` where millisecond precision is enough.
