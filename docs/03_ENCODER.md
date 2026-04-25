# 03 — MicromouseEncoder.h

## What this class does

Counts directional ticks from a single-channel quadrature encoder using one
hardware interrupt. Provides `getTicks()` and `reset()` to the PID loop.

---

## Decision 1 — Why `volatile long count`

```cpp
volatile long count;
```

`count` is written inside an ISR and read from the main loop. Without
`volatile`, the compiler is allowed to cache `count` in a CPU register between
reads, meaning `getTicks()` would return a stale value even as the ISR keeps
incrementing it. `volatile` tells the compiler: *this variable can change at
any time outside the normal flow of execution — always read it fresh from
memory.*

**Why `long` (32-bit) and not `int` (16-bit)?**

At 210 ticks/rev and a maximum speed of 500 RPM = 8.33 rev/sec:
- Max tick rate ≈ 1750 ticks/sec
- An `int16_t` would overflow in 32767 / 1750 ≈ **18.7 seconds** of running

`long` (32-bit signed) overflows at ±2,147,483,647, which at 1750 ticks/sec
gives **~346 hours** before overflow. For a competition run that is effectively
never.

---

## Decision 2 — Why `INPUT_PULLUP` on both pins

```cpp
pinMode(pinA, INPUT_PULLUP);
pinMode(pinB, INPUT_PULLUP);
```

The encoder outputs are open-collector or open-drain on most N20 encoder
modules. Without a pull-up the pin floats when the encoder output is
de-asserted, causing random noise triggers on the interrupt line.

`INPUT_PULLUP` uses the ESP32-S3's internal ~45 kΩ pull-up resistor, which is
sufficient for the short PCB traces on a Micromouse. If you see random spurious
counts, add an external 10 kΩ pull-up and a 10 nF decoupling cap per encoder
pin — the internal pull-up may not be strong enough at high speeds.

---

## Decision 3 — RISING edge only (single-channel mode)

```cpp
attachInterrupt(digitalPinToInterrupt(pinA), ISR_callback, RISING);
```

**Why RISING only, not CHANGE?**

This is a deliberate trade-off: fewer interrupts per revolution in exchange for
interrupt rate safety.

Full quadrature (CHANGE on both channels A and B) gives 4× the resolution
(7 PPR × 4 × 30 = 840 ticks/rev) but also 4× the interrupt rate. At
500 RPM = 8.33 rev/sec, full quadrature fires at 840 × 8.33 = **~7000 ISR
calls/sec**. With PID at 50 Hz, I2C IMU reads, and Serial output all competing,
this risks starving the main loop.

RISING-only on one channel fires at 210 × 8.33 = **~1750 ISR/sec** — much
more manageable and leaves headroom for maze-solving logic later.

**To upgrade to full quadrature:**
1. Change `RISING` to `CHANGE`
2. Also `attachInterrupt` on `pinB` with its own ISR
3. Use a 4-state lookup table for direction (A_prev, B_prev → A_new, B_new)
4. Update `TICKS_PER_REV` in `PinConfig.h` to 840

---

## Decision 4 — Direction detection: read pinB at the moment of pinA RISING

```cpp
void IRAM_ATTR handleInterrupt() {
    digitalRead(pinB) ? count++ : count--;
}
```

**Why this works:**

For a standard quadrature encoder, channel B leads channel A by 90° in the
forward direction and lags by 90° in the reverse direction. At the RISING edge
of A:
- If B is HIGH → forward → increment
- If B is LOW  → reverse → decrement

This gives direction information from a single interrupt source without
needing a second interrupt.

**Known limitation:** `digitalRead()` inside an ISR has a small latency (~50 ns
on ESP32-S3). At very high speeds the B signal could transition between the
A edge and the `digitalRead` call. This becomes a problem above ~2000 RPM
motor shaft speed. At 500 RPM no-load this is not an issue.

---

## Decision 5 — `IRAM_ATTR` on `handleInterrupt()`

```cpp
void IRAM_ATTR handleInterrupt() { ... }
```

The ESP32-S3 stores most code in flash (SPI). Flash reads are cached, but on
a cache miss the CPU stalls for several microseconds while the flash controller
fetches the line. If an ISR is in flash and causes a cache miss, the interrupt
latency spikes unpredictably — this corrupts time-sensitive encoder counts.

`IRAM_ATTR` forces the function into IRAM (internal RAM), which has zero-wait
access. ISRs should always be in IRAM on ESP32 devices.

---

## Decision 6 — ISR wrapper in `.ino`, not a static member in the class

```cpp
// In motor-control-test.ino:
void IRAM_ATTR leftISR() { leftEnc.handleInterrupt(); }
```

`attachInterrupt()` requires a plain C function pointer — it cannot accept a
member function pointer because member functions have a hidden `this` parameter.
Two options exist:

1. **Static member function** — works but requires a global/static reference
   back to the object, coupling the class to a specific instance.
2. **Free wrapper function in `.ino`** — clean separation: the class is
   completely instance-agnostic, the sketch file owns the binding.

Option 2 was chosen because it keeps `MicromouseEncoder.h` reusable without
modification when you add the right encoder.

---

## Tick math reference

```
Motor shaft: 7 pulses/rev (encoder disc)
Gear ratio:  1:30 (output shaft turns 30× slower than motor shaft)
Wheel shaft ticks/rev (RISING only, 1 channel): 7 × 30 = 210
Wheel shaft ticks/rev (full quadrature):         7 × 4 × 30 = 840

At 500 RPM motor (free-run, 6 V):
  Output shaft = 500 / 30 = 16.7 RPM = 0.278 rev/sec
  Tick rate (single RISING) = 0.278 × 210 = 58.3 ticks/sec at wheel shaft

Wait — the above is wheel shaft speed. The PID target of 800 ticks/sec implies:
  800 / 210 = 3.81 rev/sec = 228 RPM at output shaft
  Motor shaft speed = 228 × 30 = 6840 RPM ← exceeds 500 RPM no-load!

ACTION REQUIRED: Recalculate target_speed in motor-control-test.ino.
See 05_PID_LOOP.md → Decision 3 for the corrected value.
```
