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
#include "Tuning.h"

struct IRPair { uint8_t emit, rx; };

static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L45, RX_L45 },
    { EMIT_R45, RX_R45 },
    { EMIT_RF, RX_RF },
};

static int irVal[4] = { 0, 0, 0, 0 };
// Last emitter-OFF ambient per sensor — used to detect ADC saturation (under
// bright light the lit−ambient delta collapses and the read is untrustworthy).
static int irAmb[4] = { 0, 0, 0, 0 };

// EMA + edge state — read/written by the PH_FORWARD executor in main.cpp.
static float irLSm   = 0.0f, irRSm   = 0.0f;
static float irLPrev = 0.0f, irRPrev = 0.0f;
static bool  irFirstSample = true;

// Side-IR cal targets (centering setpoints). Default to PinConfig values.
static int calL = IR_CAL_L45;
static int calR = IR_CAL_R45;

// Per-RUN side WALL-detection references. Captured live in calibrateSideRefs()
// so room lighting divides out; default to the factory cal until then.
static int sideRefL = IR_CAL_L45;
static int sideRefR = IR_CAL_R45;

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

// Fill irVal[] (lit−ambient) AND irAmb[] (emitter-off baseline, for saturation).
static void sampleIR() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(PAIRS[i].emit, LOW);  delayMicroseconds(80);
        int amb = analogRead(PAIRS[i].rx);
        digitalWrite(PAIRS[i].emit, HIGH); delayMicroseconds(80);
        int lit = analogRead(PAIRS[i].rx);
        digitalWrite(PAIRS[i].emit, LOW);
        irAmb[i] = amb;
        int d = lit - amb;
        irVal[i] = d < 0 ? 0 : d;
    }
}

static int medianOfN(int* a, int n) {            // tiny n → insertion sort
    for (int i = 1; i < n; i++) {
        int key = a[i], j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    return a[n / 2];
}

// Robust read of the two SIDE sensors: median of SIDE_SAMPLES (beats noise on
// the weak channel) into irVal[1]/irVal[2], with the WORST-case ambient into
// irAmb[1]/irAmb[2] (so a saturation spike in any sample is caught).
static void sampleSideMedian() {
    int n = SIDE_SAMPLES; if (n < 1) n = 1; if (n > 9) n = 9;
    int bufL[9], bufR[9];
    int ambL = 0, ambR = 0;
    for (int k = 0; k < n; k++) {
        sampleIR();
        bufL[k] = irVal[1]; bufR[k] = irVal[2];
        if (irAmb[1] > ambL) ambL = irAmb[1];
        if (irAmb[2] > ambR) ambR = irAmb[2];
    }
    irVal[1] = medianOfN(bufL, n);
    irVal[2] = medianOfN(bufR, n);
    irAmb[1] = ambL; irAmb[2] = ambR;
}

#endif
