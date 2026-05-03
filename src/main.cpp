// main.cpp — LF IR pair test + BLE UART debug
// TEFT4300 behavior: high ADC = IR present, low ADC = no IR
// delta = lit - ambient  (positive = wall/object)
// BLE: connect "Micromouse26" in Serial Bluetooth Terminal app
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"

// ── BLE NUS ──────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX        = nullptr;
bool                  bleConn    = false;

void blePrintf(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    // USB serial always
    Serial.print(buf);
    // BLE in 20-byte chunks
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

// ── setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(EMIT_LF, OUTPUT);
    digitalWrite(EMIT_LF, LOW);
    pinMode(RX_LF, INPUT);

    bleSetup();

    blePrintf("\n[LF-TEST] emit=GPIO%d rx=GPIO%d\n", EMIT_LF, RX_LF);
    blePrintf("[LF-TEST] delta=ambient-lit, positive=wall\n\n");
}

// ── loop ─────────────────────────────────────────────────────────────────────
void loop() {
    int ambient = analogRead(RX_LF);

    digitalWrite(EMIT_LF, HIGH);
    delayMicroseconds(100);
    int lit = analogRead(RX_LF);
    digitalWrite(EMIT_LF, LOW);

    int delta = lit - ambient;  // TEFT4300: lit = higher ADC when wall present

    blePrintf("amb=%4d lit=%4d d=%4d %s\n",
              ambient, lit, delta,
              delta < 50  ? "open" :
              delta < 300 ? "far"  :
              delta < 800 ? "near" : "WALL");
    delay(100);
}
