# IR Sensor Test — SFH4545 + TEFT4300

## Hardware (4 sensors active)

| Sensor | Emitter pin | Receiver pin |
|--------|-------------|--------------|
| LF (Left-Front)  | GPIO13 | GPIO4  |
| L45 (Left-45)    | GPIO45 | GPIO6  |
| R45 (Right-45)   | GPIO12 | GPIO2  |
| RF (Right-Front) | GPIO11 | GPIO1  |

L-Side (GPIO47/GPIO10) and R-Side (GPIO46/GPIO7) pins defined in PinConfig.h but not soldered — add later if needed.

## Circuit

**SFH4545 emitter (5V driven via 8050 NPN):**
```
5V → SFH4545 anode → cathode → 8050 collector → emitter → GND
GPIO → 330Ω → 8050 base   (HIGH = transistor on = IR fires)
```

**TEFT4300 receiver (3.3V):**
```
3.3V → 1.8kΩ → collector → GPIO (ADC)
emitter → GND
```
Note: TEFT4300 installed with collector/emitter swapped on PCB.
Actual behavior: **high ADC = IR present, low ADC = no IR**

## Differential reading
```
delta = lit - ambient   (positive = wall/object detected)
```

## Thresholds (tuned on actual maze)
| delta | Status |
|-------|--------|
| < 50  | open   |
| < 300 | far    |
| < 800 | near   |
| ≥ 800 | WALL   |

Update `IR_THRESH_FRONT` and `IR_THRESH_DIAG` in `PinConfig.h` after maze calibration.

## main.cpp

```cpp
// main.cpp — 4x IR sensor test + BLE UART debug
// TEFT4300: high ADC = IR present, low ADC = no IR
// delta = lit - ambient  (positive = wall)
// BLE: connect "Micromouse26" in Serial Bluetooth Terminal
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;

void blePrintf(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    if (!bleConn || !pTX) return;
    size_t len = strlen(buf), off = 0;
    while (off < len) {
        size_t chunk = min((size_t)20, len - off);
        pTX->setValue((uint8_t*)(buf + off), chunk);
        pTX->notify();
        off += chunk;
        delay(10);
    }
}

class BLECb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*)    override { bleConn = true;  Serial.println("[BLE] connected");    }
    void onDisconnect(NimBLEServer*) override { bleConn = false; Serial.println("[BLE] disconnected"); NimBLEDevice::startAdvertising(); }
};

void bleSetup() {
    NimBLEDevice::init("Micromouse26");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    auto* srv = NimBLEDevice::createServer();
    srv->setCallbacks(new BLECb());
    auto* svc = srv->createService(NUS_SERVICE_UUID);
    pTX = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    svc->createCharacteristic(NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    svc->start();
    NimBLEDevice::getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] advertising — connect to 'Micromouse26'");
}

struct IRPair { const char* name; uint8_t emit; uint8_t rx; };
static const IRPair PAIRS[] = {
    { "LF ", EMIT_LF,  RX_LF  },
    { "L45", EMIT_L45, RX_L45 },
    { "R45", EMIT_R45, RX_R45 },
    { "RF ", EMIT_RF,  RX_RF  },
};
static const int N = 4;

int readDelta(const IRPair& p) {
    int ambient = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(100);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    return lit - ambient;
}

const char* classify(int delta) {
    if (delta < 50)  return "open";
    if (delta < 300) return "far";
    if (delta < 800) return "near";
    return "WALL";
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    for (int i = 0; i < N; i++) {
        pinMode(PAIRS[i].emit, OUTPUT);
        digitalWrite(PAIRS[i].emit, LOW);
        pinMode(PAIRS[i].rx, INPUT);
    }
    bleSetup();
    blePrintf("\n[IR-TEST] LF L45 R45 RF\n\n");
}

void loop() {
    int d[N];
    for (int i = 0; i < N; i++) d[i] = readDelta(PAIRS[i]);
    blePrintf("LF:%4d %s | L45:%4d %s | R45:%4d %s | RF:%4d %s\n",
              d[0], classify(d[0]),
              d[1], classify(d[1]),
              d[2], classify(d[2]),
              d[3], classify(d[3]));
    delay(200);
}
```

## Notes
- L-Side and R-Side pins defined in PinConfig.h, not soldered — extend N to 6 and add pairs when ready
- Recalibrate thresholds at actual maze wall distance (~4cm)
- BLE chunks in 20-byte MTU — `blePrintf` handles splitting automatically
