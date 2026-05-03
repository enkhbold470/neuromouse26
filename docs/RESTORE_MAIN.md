# Restore main.cpp — full firmware checklist

When restoring from LED test back to full firmware:

## Changes vs original main.cpp

### New includes
- `#include <FastLED.h>`

### New globals
- `#define NUM_LEDS 1`
- `CRGB leds[NUM_LEDS];`

### platformio.ini
- `build_flags = -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1`
- `lib_deps = fastled/FastLED @ ^3.6.0`

### FSM additions
- `STATE_SELFTEST = 6` in enum
- `STATE_NAME[7]` (was [6])
- `case STATE_SELFTEST: doSelfTest(); break;` in loop()
- `void doSelfTest();` forward decl
- `doIdle()` → transitions to `STATE_SELFTEST` (not `STATE_CALIBRATE`)

### setup() additions
- `delay(2000)` (was 500) for CDC enumerate
- FastLED init block after buzzer init
- Updated IDLE print message

### New functions
- `ledFlash(CRGB, int, int, int)` — static helper
- `waitForButtonPress()` — static helper
- `waitForButtonShortOrLong()` — static helper, returns bool
- `doSelfTest()` — 10-step peripheral self-test, blocking

### Serial spam removed
- `imu.update()` — verbose printf stripped (was firing every loop)
- TODO on restore: strip `coast()`/`brake()`/`drive()` per-call prints from MicromouseMotor.h

### Bug fixes applied
- `PinConfig.h`: `PWM_FREQ` → `MOTOR_PWM_FREQ`, `PWM_RES` → `MOTOR_PWM_RES`
- `MicromouseMaze.h:145`: removed `F()` from `Serial.printf`
- `MicromouseEncoder.h`: `handleInterrupt()` declaration only — body moved to `src/MicromouseEncoder.cpp`
- `MicromouseIMU.h`: added `getBias() const`

### New file
- `src/MicromouseEncoder.cpp` — out-of-line IRAM_ATTR handleInterrupt()

## Motor verbose prints to strip on restore
In `MicromouseMotor.h` remove Serial.printf from:
- `begin()` — keep first line only (init summary), remove per-channel lines
- `drive()` — remove entirely
- `coast()` — remove entirely  
- `brake()` — remove entirely
