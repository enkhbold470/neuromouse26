// main.cpp — BLE UART (Nordic UART Service) test
// Mirrors Serial output to BLE notify characteristic
// App: "Serial Bluetooth Terminal" (Android/iOS) — connect to "Micromouse26"
// NUS UUIDs (standard Nordic UART Service):
//   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   TX (notify, ESP→phone) : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
//   RX (write, phone→ESP)  : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"

// ── NUS UUIDs ────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer*         pServer   = nullptr;
NimBLECharacteristic* pTX       = nullptr;
bool                  bleConnected = false;

// ── Send string over BLE TX (splits into 20-byte MTU chunks) ─────────────────
void blePrint(const char* str) {
    if (!bleConnected || !pTX) return;
    size_t len = strlen(str);
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = min((size_t)20, len - offset);
        pTX->setValue((uint8_t*)(str + offset), chunk);
        pTX->notify();
        offset += chunk;
        delay(10);  // avoid flooding BLE stack
    }
}

void blePrintf(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    blePrint(buf);
}

// ── Server callbacks ──────────────────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s) override {
        bleConnected = true;
        Serial.println("[BLE] client connected");
    }
    void onDisconnect(NimBLEServer* s) override {
        bleConnected = false;
        Serial.println("[BLE] client disconnected — restarting advertising");
        NimBLEDevice::startAdvertising();
    }
};

// ── RX callback — phone → ESP ─────────────────────────────────────────────────
class RXCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) override {
        std::string val = c->getValue();
        if (!val.empty()) {
            Serial.printf("[BLE-RX] received: %s\n", val.c_str());
            // Echo back
            blePrintf("echo: %s\n", val.c_str());
        }
    }
};

// ── setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[BLE-TEST] ==============================");
    Serial.println("[BLE-TEST] NimBLE UART (NUS) test");
    Serial.println("[BLE-TEST] Device name: Micromouse26");
    Serial.println("[BLE-TEST] App: Serial Bluetooth Terminal");
    Serial.println("[BLE-TEST] ==============================\n");

    NimBLEDevice::init("Micromouse26");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // max TX power

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(NUS_SERVICE_UUID);

    // TX characteristic — ESP notifies phone
    pTX = pService->createCharacteristic(
        NUS_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // RX characteristic — phone writes to ESP
    NimBLECharacteristic* pRX = pService->createCharacteristic(
        NUS_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRX->setCallbacks(new RXCallbacks());

    pService->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->start();

    Serial.println("[BLE] advertising started — connect with phone app");
}

// ── loop ─────────────────────────────────────────────────────────────────────
void loop() {
    static unsigned long last = 0;
    static uint32_t      count = 0;

    if (millis() - last >= 1000) {
        last = millis();
        count++;

        // Print to USB serial always
        Serial.printf("[TICK] %lu  ble=%s\n", count, bleConnected ? "connected" : "waiting...");

        // Print to BLE when connected
        if (bleConnected) {
            blePrintf("[TICK] %lu\n", count);
        }
    }
}
