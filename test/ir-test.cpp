// test/ir-test.cpp — IR delta direction diagnostic
//
// Prints both (amb-lit) and (lit-amb) per sensor.
// Hold white paper ~3cm from each sensor while watching serial.
// Whichever column goes POSITIVE when wall is close = correct formula.
//
// Hardware: LF=IO13/IO4  RF=IO11/IO1  R45=IO12/IO2  L45=IO45/IO6
//
// Flash: pio run -e ir-test -t upload

#include <Arduino.h>

struct IRPair { const char* name; uint8_t emit, rx; };

static const IRPair PAIRS[4] = {
    { "LF ", 13, 4  },
    { "RF ", 11, 1  },
    { "R45", 12, 2  },
    { "L45", 45, 6  },
};

void setup() {
    Serial.begin(115200);
    delay(500);
    analogReadResolution(12);
    for (const auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    Serial.println("\n=== IR DIRECTION TEST — hold paper ~3cm from sensor ===");
    Serial.println("Name  emit rx   amb   lit  A-L  L-A");
    Serial.println("======================================");
}

void loop() {
    for (const auto& p : PAIRS) {
        digitalWrite(p.emit, LOW);
        delayMicroseconds(80);
        int amb = analogRead(p.rx);
        digitalWrite(p.emit, HIGH);
        delayMicroseconds(80);
        int lit = analogRead(p.rx);
        digitalWrite(p.emit, LOW);
        Serial.printf("%s  IO%-2d IO%-2d  %4d  %4d  %4d  %4d\n",
            p.name, p.emit, p.rx, amb, lit,
            max(0, amb - lit), max(0, lit - amb));
    }
    Serial.println("---");
    delay(300);
}
