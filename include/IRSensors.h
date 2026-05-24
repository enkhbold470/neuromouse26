// include/IRSensors.h — 4-sensor IR array (LF, L, R, RF).
//
// Differential ambient/lit read in readIR(). sampleIR() fills irVal[4].
// Indices: 0=LF, 1=L, 2=R, 3=RF.
//
// `irFirstSample` is reset by scriptKick() (in main.cpp) so the EMA inside
// the PH_FORWARD executor reseeds at every new script start.

#ifndef MM26_IR_SENSORS_H
#define MM26_IR_SENSORS_H

#include <Arduino.h>
#include "PinConfig.h"

struct IRPair { uint8_t emit, rx; };

static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L45, RX_L45 },
    { EMIT_R45, RX_R45 },
    { EMIT_RF, RX_RF },
};

static int irVal[4] = { 0, 0, 0, 0 };

// EMA + edge state — read/written by the PH_FORWARD executor in main.cpp.
static float irLSm   = 0.0f, irRSm   = 0.0f;
static float irLPrev = 0.0f, irRPrev = 0.0f;
static bool  irFirstSample = true;

// Side-IR cal targets. Default to PinConfig values; can be overridden at
// runtime by calibration sketches if needed.
static int calL = IR_CAL_L45;
static int calR = IR_CAL_R45;

static int readIR(const IRPair& p) {
    digitalWrite(p.emit, LOW);
    delayMicroseconds(80);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(80);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    int d = lit - amb;
    return d < 0 ? 0 : d;
}

static void sampleIR() { for (int i = 0; i < 4; i++) irVal[i] = readIR(PAIRS[i]); }

#endif
