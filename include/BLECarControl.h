// include/BLECarControl.h — RC-car-style BLE control mode.
//
// Compatible with the "BLE Controller" app by CircuitMagic (Android). The app
// sends plain UTF-8 strings over Nordic UART Service (NUS) — same UUIDs as
// test/ble-test.cpp.
//
// Default button commands the app sends:
//   F = forward   B = backward   L = left   R = right   S = stop
//
// Holding a button typically only sends one press event, so commands LATCH —
// the robot keeps doing whatever the last command said until a new one
// arrives. Configure the app's release event to send "S" if you want
// dead-man-switch behavior, or use a watchdog timeout below.
//
// All NimBLE state lives in this header (single-TU project). `bleCarInit()`
// is idempotent — calling it a second time is a no-op.
//
// The main loop drives motors from `bleCarLastCmd` directly; this header
// owns nothing that touches motors or encoders.

#ifndef MM26_BLE_CAR_CONTROL_H
#define MM26_BLE_CAR_CONTROL_H

#include <Arduino.h>
#include <NimBLEDevice.h>

namespace BLECar {

constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char RX_UUID[]      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char TX_UUID[]      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// Safety watchdog: if no BLE command received within this window,
// stop motors to prevent runaway on disconnect/phone crash.
constexpr uint32_t WATCHDOG_MS = 500;

static bool                  initialized   = false;
static bool                  connected     = false;
static char                  lastCmd       = 'S';
static uint32_t              lastCmdMs     = 0;
static NimBLEServer*         server        = nullptr;
static NimBLECharacteristic* txChar        = nullptr;

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        connected = true;
        Serial.println("[BLE-CAR] client connected");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        connected = false;
        lastCmd   = 'S';
        Serial.println("[BLE-CAR] client disconnected — re-advertising");
        NimBLEDevice::startAdvertising();
    }
};

class RxCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        std::string v = c->getValue();
        if (v.empty()) return;
        // Take first non-whitespace ASCII char, upcased. App sends short
        // strings like "F", "B"; for compound strings ("FL"), we just use
        // the first letter — single-axis control is plenty for this chassis.
        char ch = 0;
        for (char x : v) {
            if (x > ' ') { ch = x; break; }
        }
        if (ch >= 'a' && ch <= 'z') ch -= 32;
        if (ch == 'F' || ch == 'B' || ch == 'L' || ch == 'R' || ch == 'S') {
            lastCmd   = ch;
            lastCmdMs = millis();
            Serial.printf("[BLE-CAR] cmd=%c\n", ch);
        } else {
            Serial.printf("[BLE-CAR] unknown cmd \"%s\"\n", v.c_str());
        }
    }
};

inline void init() {
    if (initialized) {
        NimBLEDevice::startAdvertising();
        return;
    }
    NimBLEDevice::init("Micromouse26");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCB());

    NimBLEService* svc = server->createService(SERVICE_UUID);
    txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* rx = svc->createCharacteristic(
        RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RxCB());

    // NimBLE 2.x: NimBLEService::start() is a no-op; the GATT server is
    // started lazily by startAdvertising() (NimBLEAdvertising.cpp:201).
    //
    // ALSO 2.x: the device name from NimBLEDevice::init() is NOT auto-added
    // to the advertisement — Web Bluetooth's namePrefix filter sees a
    // nameless broadcast otherwise. setName + enableScanResponse fix it.
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName("Micromouse26");
    adv->addServiceUUID(SERVICE_UUID);
    adv->enableScanResponse(true);
    adv->start();

    initialized = true;
    Serial.println("[BLE-CAR] advertising as Micromouse26 (NUS)");
}

inline void notify(const char* msg) {
    if (!connected || !txChar) return;
    txChar->setValue((uint8_t*)msg, strlen(msg));
    txChar->notify();
}

}  // namespace BLECar

#endif
