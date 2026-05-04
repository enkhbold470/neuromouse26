# 03 — MicromouseEncoder.h

## What it does

Counts directional ticks from single-channel quadrature encoder via one hardware interrupt. Provides `getTicks()` + `reset()` to PID loop.

---

## Decision 1 — `volatile long count`

```cpp
volatile long count;
```

Written in ISR, read from main loop. Without `volatile` compiler caches value in register → stale reads.

`long` (32-bit): at 1750 ticks/sec, `int16_t` overflows in 18.7s. `long` overflows in ~346 hours — effectively never.

---

## Decision 2 — `INPUT_PULLUP` on both pins

Encoder outputs open-collector — without pullup pin floats → random noise triggers. Internal ~45kΩ sufficient for short PCB traces. If spurious counts appear: add external 10kΩ + 10nF cap per pin.

---

## Decision 3 — RISING only (single-channel)

```cpp
attachInterrupt(digitalPinToInterrupt(pinA), ISR_callback, RISING);
```

Full quadrature (CHANGE both channels) = 840 ticks/rev but 7000 ISR/sec at 500RPM — risks starving main loop.
RISING-only = 210 ticks/rev, 1750 ISR/sec — manageable.

**Upgrade to full quadrature:**
1. Change `RISING` → `CHANGE`
2. Attach interrupt on `pinB`
3. Use 4-state lookup table (A_prev, B_prev → A_new, B_new)
4. Update `TICKS_PER_REV` → 840

---

## Decision 4 — Direction: read pinB at pinA RISING

```cpp
void IRAM_ATTR handleInterrupt() {
    digitalRead(pinB) ? count++ : count--;
}
```

B leads A by 90° forward, lags 90° reverse. At RISING edge of A: B HIGH = forward, B LOW = reverse.

**Limitation:** `digitalRead` ~50ns latency. Above ~2000RPM motor shaft B could transition before read. Fine at 500RPM.

---

## Decision 5 — `IRAM_ATTR`

```cpp
void IRAM_ATTR handleInterrupt() { ... }
```

Code in flash → cache miss = multi-µs stall → corrupted encoder counts. `IRAM_ATTR` forces into IRAM = zero-wait. All ISRs on ESP32 must be in IRAM.

---

## Decision 6 — ISR wrapper in `.ino`, not static member

```cpp
void IRAM_ATTR leftISR() { leftEnc.handleInterrupt(); }
```

`attachInterrupt()` needs plain C function pointer — no member function pointers (hidden `this`). Wrapper in sketch keeps class instance-agnostic and reusable.

---

## Tick math

```
Motor shaft: 7 pulses/rev
Gear ratio:  1:30
Wheel ticks/rev (RISING, 1ch): 7 × 30 = 210
Wheel ticks/rev (full quadrature): 7 × 4 × 30 = 840

At 500RPM output shaft = 8.33 rev/sec:
  Single RISING: 8.33 × 210 = 1750 ticks/sec
```
