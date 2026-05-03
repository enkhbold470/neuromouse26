// MicromouseEncoder.cpp
// Provides out-of-line definition of IRAM_ATTR handleInterrupt().
// Must NOT be in the header — inline IRAM_ATTR methods cause
// "literal placed after use" linker errors on Xtensa ESP32.
#include "MicromouseEncoder.h"

void IRAM_ATTR MicromouseEncoder::handleInterrupt() {
    // B HIGH when A rises → forward (count++)
    // B LOW  when A rises → reverse (count--)
    digitalRead(pinB) ? count++ : count--;
}
