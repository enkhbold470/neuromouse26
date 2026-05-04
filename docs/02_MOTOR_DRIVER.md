# 02 — MicromouseMotor.h

## What it does

Abstracts one DRV8833 H-bridge channel. PID calls `drive(int speed)` −1023 to +1023, never touches GPIO/ledc directly.

---

## Decision 1 — PWM freq: 20kHz

```cpp
const uint32_t PWM_FREQ = 20000;
```

- Below ~18kHz → audible whine from coil + FETs
- Above ~30kHz → switching loss increases, efficiency drops
- 20kHz = industry sweet-spot for small brushed DC

---

## Decision 2 — PWM res: 10-bit (0–1023)

```cpp
const uint8_t PWM_RES = 10;
```

- 8-bit = 255 steps, PID integral jumps cause visible speed bumps
- 12-bit at 20kHz: `20000 × 4096 = 81.9MHz` — exceeds 80MHz APB clock → garbage PWM
- 10-bit: `20000 × 1024 = 20.5MHz` — safe

---

## Decision 3 — `MIN_POWER = 150` (friction feedforward)

```cpp
const int MIN_POWER = 150;
```

Brushed DC has stiction deadband — PWM too low = motor doesn't move, PID integral winds up.

```cpp
int pwm = map(speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
```

Remaps [1,1023] → [150,1023]. PID never sends below 150 when intending to move.

Tune per motor: lower if stuttering, raise if lurching from stop.
**Original bug:** `MIN_POWER = 40` → 0.23V → motor never moved.

---

## Decision 4 — Fast decay (coast) vs slow decay (brake)

```
Forward:  IN1 = PWM,  IN2 = 0    → Fast Decay Forward
Reverse:  IN1 = 0,   IN2 = PWM   → Fast Decay Reverse
Brake:    IN1 = HIGH, IN2 = HIGH  → Slow Decay (active brake)
Coast:    IN1 = 0,   IN2 = 0     → Fast Decay Zero (freewheeling)
```

- Fast decay = smooth average torque, linear PWM→speed (what PID assumes)
- Slow decay = short-circuits windings, rapid stop via back-EMF

**Original bug:** `brake()` and `coast()` swapped — active braking on PID zero output, motor resisted all movement.

---

## Decision 5 — Constructor init list

```cpp
MicromouseMotor(uint8_t in1, uint8_t in2) : pinIN1(in1), pinIN2(in2) {}
```

Idiomatic C++, signals members set at construction and not mutated.

---

## API

| Method | Input | Effect |
|---|---|---|
| `begin()` | none | Attach ledc to pins, call `coast()` |
| `drive(int speed)` | −1023 to +1023 | Set PWM; 0 → `coast()` |
| `brake()` | none | Both pins HIGH → slow decay stop |
| `coast()` | none | Both pins LOW → freewheeling |
