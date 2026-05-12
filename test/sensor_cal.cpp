// test/sensor_cal.cpp — pro-style IR sampler
// FreeRTOS task on Core 0 scans all 4 sensors continuously.
// Main loop (Core 1) just prints latest values @ 20 Hz — no blocking.
//
// Settle time tightened to 80µs (SFH4545 rise ≈10µs, BPV23 fall ≈30µs).
// Full scan ≈ 4 × (2 × analogRead + 80µs) ≈ 800µs → ~1.2 kHz per sensor.
//
// delta = max(0, amb - lit). Closer wall → bigger value.
// Order: L, LF, RF, R. Cmd 'r' resets peaks.

#include <Arduino.h>
#include "../include/PinConfig.h"

struct IRPair { const char* name; uint8_t emit, rx; };

static IRPair PAIRS[4] = {
    { "L ", EMIT_L,  RX_L  },
    { "LF", EMIT_LF, RX_LF },
    { "RF", EMIT_RF, RX_RF },
    { "R ", EMIT_R,  RX_R  },
};

// Shared between sampler task and printer. volatile = no caching across cores.
volatile int      irVal[4]   = {0,0,0,0};
volatile int      irMin[4]   = {INT_MAX,INT_MAX,INT_MAX,INT_MAX};
volatile int      irMax[4]   = {0,0,0,0};
volatile uint32_t scanCount  = 0;
volatile bool     wantReset  = false;

static inline int readDelta(const IRPair& p) {
    digitalWrite(p.emit, LOW);
    delayMicroseconds(80);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(80);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    int d = amb - lit;
    return d < 0 ? 0 : d;
}

// Sampler task — Core 0, max rate
void samplerTask(void*) {
    for (;;) {
        if (wantReset) {
            for (int i = 0; i < 4; i++) { irMin[i] = INT_MAX; irMax[i] = 0; }
            scanCount = 0;
            wantReset = false;
        }
        for (int i = 0; i < 4; i++) {
            int v = readDelta(PAIRS[i]);
            irVal[i] = v;
            if (v < irMin[i]) irMin[i] = v;
            if (v > irMax[i]) irMax[i] = v;
        }
        scanCount++;
        // No vTaskDelay — sensors need fastest scan. Yield via taskYIELD() so
        // FreeRTOS housekeeping still runs on Core 0.
        taskYIELD();
    }
}

void setup() {
    Serial.begin(115200);
    delay(800);
    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);

    Serial.println("\n--- IR SENSOR CAL (Core0 sampler) ---");
    Serial.println("Format: cur (min/max)  rate=Hz  — closer wall = bigger");
    Serial.println("Cmd: 'r' reset stats");
    for (auto& p : PAIRS)
        Serial.printf("  %s emit=GPIO%-2d rx=GPIO%-2d\n", p.name, p.emit, p.rx);
    Serial.println("-------------------------------------\n");

    xTaskCreatePinnedToCore(samplerTask, "ir-samp", 4096, nullptr, 5, nullptr, 0);
}

void loop() {
    if (Serial.available() && Serial.read() == 'r') wantReset = true;

    static uint32_t lastCnt = 0;
    static uint32_t lastMs  = 0;
    uint32_t now = millis();
    if (now - lastMs >= 100) {
        uint32_t cnt  = scanCount;
        float    hz   = (cnt - lastCnt) * 1000.0f / (now - lastMs);
        lastCnt = cnt; lastMs = now;
        for (int i = 0; i < 4; i++) {
            Serial.printf("%s %4d (%4d/%4d) ",
                          PAIRS[i].name, irVal[i], irMin[i], irMax[i]);
        }
        Serial.printf(" %.0fHz n=%lu\n", hz, (unsigned long)cnt);
    }
}
