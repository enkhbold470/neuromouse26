// MicromouseEncoder.cpp
// IRAM_ATTR definition must be out-of-line on Xtensa ESP32-S3.
#include "MicromouseEncoder.h"

void IRAM_ATTR MicromouseEncoder::handleInterrupt() {
    digitalRead(pinB) ? count++ : count--;
}
