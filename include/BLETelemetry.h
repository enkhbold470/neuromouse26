// include/BLETelemetry.h — BLE UART telemetry for real-time robot monitoring.
//
// Nordic UART Service (NUS) UUIDs — same as tools/ble-debug.html.
// TX (notify) → browser  |  RX (write) ← browser commands.
//
// Call bleInit() in setup().
// Call bleGetCmd() in loop() to process incoming commands.
// Motion telemetry is self-gated to 10 Hz.

#ifndef MM26_BLE_TELEMETRY_H
#define MM26_BLE_TELEMETRY_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "IRSensors.h"
#include "Pose.h"

#define BLE_SVC_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TX_UUID   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_RX_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

static NimBLEServer*         g_bleServer = nullptr;
static NimBLECharacteristic* g_bleTx     = nullptr;
static NimBLECharacteristic* g_bleRx     = nullptr;
static bool                  g_bleConn   = false;
static uint32_t              g_bleMotT   = 0;

static char g_bleCmd[64]  = {0};
static bool g_bleCmdReady = false;

struct BLESrvCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        g_bleConn = true;
        s->updateConnParams(info.getConnHandle(), 6, 12, 0, 300);
        Serial.println("[BLE] connected");
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        g_bleConn = false;
        NimBLEDevice::getAdvertising()->start();
        Serial.println("[BLE] disconnected, advertising");
    }
};

struct BLERxCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
        NimBLEAttValue v = c->getValue();
        size_t len = v.length();
        if (len > 0 && len < sizeof(g_bleCmd)) {
            memcpy(g_bleCmd, v.data(), len);
            g_bleCmd[len] = '\0';
            g_bleCmdReady = true;
        }
    }
};

static void bleInit() {
    NimBLEDevice::init("bromouse");
    NimBLEDevice::setPower(9);  // max TX power for reliable discovery

    g_bleServer = NimBLEDevice::createServer();
    g_bleServer->setCallbacks(new BLESrvCB());

    NimBLEService* svc = g_bleServer->createService(BLE_SVC_UUID);

    g_bleTx = svc->createCharacteristic(BLE_TX_UUID,
                  NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    g_bleRx = svc->createCharacteristic(BLE_RX_UUID,
                  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    g_bleRx->setCallbacks(new BLERxCB());

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->start();

    Serial.println("[BLE] advertising as 'bromouse'");
}

static void bleSend(const char* msg) {
    if (!g_bleConn || !g_bleTx) return;
    g_bleTx->setValue((const uint8_t*)msg, strlen(msg));
    g_bleTx->notify();
}

static void bleSendf(const char* fmt, ...) {
    if (!g_bleConn) return;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    bleSend(buf);
}

static void bleState(const char* s) {
    bleSendf("{\"t\":\"ST\",\"v\":\"%s\"}", s);
}

static void blePos(int r, int c, int h) {
    bleSendf("{\"t\":\"POS\",\"r\":%d,\"c\":%d,\"h\":\"%c\"}", r, c, "NESW"[h]);
}

static void bleWall(int r, int c, int h, uint8_t walls, bool wF, bool wL, bool wR) {
    bleSendf("{\"t\":\"WALL\",\"r\":%d,\"c\":%d,\"h\":\"%c\",\"w\":%d,\"f\":%d,\"l\":%d,\"rv\":%d}",
             r, c, "NESW"[h], walls, wF?1:0, wL?1:0, wR?1:0);
}

static void bleMotion(const char* phase, long target, long avg,
                      int pwmL, int pwmR, float yaw) {
    uint32_t now = millis();
    if (now - g_bleMotT < 100) return;
    g_bleMotT = now;
    bleSendf("{\"t\":\"MOT\",\"ph\":\"%s\",\"tg\":%ld,\"av\":%ld,"
             "\"pL\":%d,\"pR\":%d,\"yw\":%.1f,"
             "\"lf\":%d,\"l\":%d,\"r\":%d,\"rf\":%d}",
             phase, target, avg, pwmL, pwmR, yaw,
             irVal[0], irVal[1], irVal[2], irVal[3]);
}

static void bleBat(int pct) {
    bleSendf("{\"t\":\"BAT\",\"v\":%d}", pct);
}

static void bleCrash(int r, int c, int h, uint8_t walls) {
    bleSendf("{\"t\":\"CRASH\",\"r\":%d,\"c\":%d,\"h\":\"%c\",\"w\":%d}",
             r, c, "NESW"[h], walls);
}

static void bleMazeDump(MicromouseMaze& maze) {
    if (!g_bleConn) return;
    char buf[256];
    int  len = 0;
    len += snprintf(buf + len, sizeof(buf) - len, "{\"t\":\"MAZE\",\"w\":\"");
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 3; c++)
            len += snprintf(buf + len, sizeof(buf) - len, "%02X", maze.walls[r][c]);
    len += snprintf(buf + len, sizeof(buf) - len, "\",\"v\":\"");
    for (int r = 0; r < 6; r++) {
        uint8_t vb = 0;
        for (int c = 0; c < 3; c++) if (maze.visited[r][c]) vb |= (1 << c);
        len += snprintf(buf + len, sizeof(buf) - len, "%02X", vb);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "\",\"f\":[");
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 3; c++)
            len += snprintf(buf + len, sizeof(buf) - len,
                            "%d%s", maze.flood[r][c], (r==5&&c==2)?"":",");
    snprintf(buf + len, sizeof(buf) - len, "]}");
    bleSend(buf);
}

static const char* bleGetCmd() {
    if (!g_bleCmdReady) return nullptr;
    g_bleCmdReady = false;
    return g_bleCmd;
}

#endif
