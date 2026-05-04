// test/ir-test.cpp — 4x IR sensor differential read over BLE
// delta = lit - ambient  (positive = object/wall detected)
// BLE commands: none — continuous stream at 200ms interval
//
// Thresholds:
//   < 50  = open
//   < 300 = far
//   < 800 = near
//   ≥ 800 = WALL

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;

void bleSend(const char* fmt, ...) {
    char buf[128];
    va_list args; va_start(args, fmt);
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
    void onConnect(NimBLEServer*)    override { bleConn = true;  Serial.println("[BLE] connected"); }
    void onDisconnect(NimBLEServer*) override { bleConn = false; NimBLEDevice::startAdvertising(); }
};

void bleSetup() {
    NimBLEDevice::init("Micromouse26-IR");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    auto* srv = NimBLEDevice::createServer();
    srv->setCallbacks(new BLECb());
    auto* svc = srv->createService(NUS_SERVICE_UUID);
    pTX = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    svc->createCharacteristic(NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    svc->start();
    NimBLEDevice::getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
    NimBLEDevice::startAdvertising();
}

struct IRPair { const char* name; uint8_t emit; uint8_t rx; };
static const IRPair PAIRS[] = {
    { "LF ",  EMIT_LF,  RX_LF  },
    { "L45",  EMIT_L45, RX_L45 },
    { "R45",  EMIT_R45, RX_R45 },
    { "RF ",  EMIT_RF,  RX_RF  },
};
static const int N_IR = 4;

int readDelta(const IRPair& p) {
    int ambient = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(100);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    return max(0, lit - ambient);
}

const char* classify(int d) {
    if (d < 50)  return "open";
    if (d < 300) return "far ";
    if (d < 800) return "near";
    return "WALL";
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    for (int i = 0; i < N_IR; i++) {
        pinMode(PAIRS[i].emit, OUTPUT);
        digitalWrite(PAIRS[i].emit, LOW);
        pinMode(PAIRS[i].rx, INPUT);
    }
    bleSetup();
    bleSend("\n[IR-TEST] LF | L45 | R45 | RF\n\n");
}

void loop() {
    int d[N_IR];
    for (int i = 0; i < N_IR; i++) d[i] = readDelta(PAIRS[i]);
    bleSend("LF:%4d(%s) L45:%4d(%s) R45:%4d(%s) RF:%4d(%s)\n",
            d[0], classify(d[0]),
            d[1], classify(d[1]),
            d[2], classify(d[2]),
            d[3], classify(d[3]));
    delay(200);
}
