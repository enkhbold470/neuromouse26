// test/sensor-cal-ble.cpp
//
// IR sensor calibration test with BLE capture.
// Captures front-wall readings at 1..9 cm and a dead-end "center" reading
// (all 4 walls present, robot centered). All captures live in RAM and stream
// back over BLE for paste into PinConfig.h.
//
// Each capture = N_SAMP averaged samples per sensor (default 32). Min/max
// kept alongside mean for noise inspection. Live IR stream available too.
//
// BLE NUS commands (newline optional):
//   d <cm>   capture front-wall row at <cm>  (1..9, e.g. `d 3`)
//   de       capture dead-end center row (all 4 walls)
//   s        toggle live IR stream (10 Hz)
//   ?        dump all captures (human readable)
//   csv      dump all captures as CSV row-per-distance
//   r        reset all captures
//   n <int>  set samples per capture (default 32, max 200)
//
// Local controls: encoder = scroll menu, button = trigger current action.
// Menu items mirror BLE: D1..D9, DeadEnd, Reset, Dump.
//
// Build:  pio run -e sensor-cal-ble -t upload
// Device name: neuromouse-cal

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"
#include "MicromouseEncoder.h"

// ── BLE NUS UUIDs (match tools/ble-debug.html) ───────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_DEVICE_NAME   "neuromouse-cal"

// ── Capture storage ──────────────────────────────────────────────────────────
constexpr int   N_DIST       = 9;                 // 1..9 cm
constexpr int   N_SAMP_MAX   = 200;
constexpr int   IR_SETTLE_US = 80;                // SFH4545 rise ~10us, BPV23 fall ~30us
constexpr unsigned long IR_STREAM_MS    = 100;
constexpr unsigned long OLED_REDRAW_MS  = 200;
constexpr long  ENC_PER_STEP = 20;

struct IRPair { const char* name; uint8_t emit, rx; };
// Order: LF, L, R, RF (matches main.cpp / velocity-pid-ble layout)
static IRPair PAIRS[4] = {
    { "LF", EMIT_LF, RX_LF },
    { "L ", EMIT_L,  RX_L  },
    { "R ", EMIT_R,  RX_R  },
    { "RF", EMIT_RF, RX_RF },
};

struct Capture {
    bool  valid;
    int   mean[4];
    int   vmin[4];
    int   vmax[4];
};

static Capture distCaps[N_DIST];     // index 0 == 1 cm, ... index 8 == 9 cm
static Capture deadEndCap;

static int     nSamples    = 32;

// ── BLE state ────────────────────────────────────────────────────────────────
static NimBLEServer*         pServer       = nullptr;
static NimBLECharacteristic* pTX           = nullptr;
static volatile bool         bleConnected  = false;
static volatile bool         bleCmdReady   = false;
static char                  bleCmdBuf[96];

enum Action { ACT_NONE = 0, ACT_DUMP, ACT_CSV, ACT_RESET, ACT_DEAD, ACT_DIST };
static volatile int    pendingAction = ACT_NONE;
static volatile int    pendingArg    = 0;          // cm for ACT_DIST
static volatile bool   irStream      = false;

// ── Hardware ──────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
MicromouseEncoder rightEnc(ENC_R_A, ENC_R_B);     // menu scroll wheel

// ── Status ────────────────────────────────────────────────────────────────────
static char statusLine[28] = "boot";
static void setStatus(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vsnprintf(statusLine, sizeof(statusLine), fmt, args);
    va_end(args);
}

// ── BLE print ────────────────────────────────────────────────────────────────
static void blePrint(const char* str) {
    if (!bleConnected || !pTX) return;
    size_t len = strlen(str);
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = min((size_t)20, len - offset);
        pTX->setValue((uint8_t*)(str + offset), chunk);
        pTX->notify();
        offset += chunk;
        delay(5);
    }
}
static void blePrintf(const char* fmt, ...) {
    char buf[200];
    va_list args; va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    blePrint(buf);
}

// ── IR read primitives ───────────────────────────────────────────────────────
static int readDelta(const IRPair& p) {
    digitalWrite(p.emit, LOW);
    delayMicroseconds(IR_SETTLE_US);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(IR_SETTLE_US);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    int d = amb - lit;
    return d < 0 ? 0 : d;
}

static void sampleOnce(int out[4]) {
    for (int i = 0; i < 4; i++) out[i] = readDelta(PAIRS[i]);
}

// Capture: nSamples averaged reads, plus min/max per sensor.
static void doCapture(Capture& c) {
    long sum[4] = {0,0,0,0};
    int  vmin[4]  = { INT_MAX, INT_MAX, INT_MAX, INT_MAX };
    int  vmax[4]  = { 0, 0, 0, 0 };
    int  s[4];
    for (int n = 0; n < nSamples; n++) {
        sampleOnce(s);
        for (int i = 0; i < 4; i++) {
            sum[i] += s[i];
            if (s[i] < vmin[i]) vmin[i] = s[i];
            if (s[i] > vmax[i]) vmax[i] = s[i];
        }
    }
    for (int i = 0; i < 4; i++) {
        c.mean[i] = (int)(sum[i] / nSamples);
        c.vmin[i] = vmin[i];
        c.vmax[i] = vmax[i];
    }
    c.valid = true;
}

// ── Dump helpers ─────────────────────────────────────────────────────────────
static void dumpRow(const char* label, const Capture& c) {
    if (!c.valid) {
        blePrintf("%s: ---\n", label);
        return;
    }
    blePrintf("%s LF=%d(%d-%d) L=%d(%d-%d) R=%d(%d-%d) RF=%d(%d-%d)\n",
              label,
              c.mean[0], c.vmin[0], c.vmax[0],
              c.mean[1], c.vmin[1], c.vmax[1],
              c.mean[2], c.vmin[2], c.vmax[2],
              c.mean[3], c.vmin[3], c.vmax[3]);
}
static void dumpAll() {
    blePrintf("=== captures (n=%d) ===\n", nSamples);
    for (int i = 0; i < N_DIST; i++) {
        char lbl[6]; snprintf(lbl, sizeof(lbl), "d%dcm", i + 1);
        dumpRow(lbl, distCaps[i]);
    }
    dumpRow("deade", deadEndCap);
}
static void dumpCsv() {
    blePrintf("cm,LF,L,R,RF\n");
    for (int i = 0; i < N_DIST; i++) {
        if (!distCaps[i].valid) continue;
        const Capture& c = distCaps[i];
        blePrintf("%d,%d,%d,%d,%d\n",
                  i + 1, c.mean[0], c.mean[1], c.mean[2], c.mean[3]);
    }
    if (deadEndCap.valid) {
        blePrintf("DE,%d,%d,%d,%d\n",
                  deadEndCap.mean[0], deadEndCap.mean[1],
                  deadEndCap.mean[2], deadEndCap.mean[3]);
    }
}
static void resetAll() {
    for (int i = 0; i < N_DIST; i++) distCaps[i].valid = false;
    deadEndCap.valid = false;
}

// ── BLE command parser ───────────────────────────────────────────────────────
static void handleBleCommand(const char* cmd) {
    char key[16] = {0};
    char val[48] = {0};
    int n = sscanf(cmd, "%15s %47s", key, val);
    if (n < 1 || key[0] == 0) return;

    if      (!strcmp(key, "?"))    { pendingAction = ACT_DUMP;  return; }
    else if (!strcmp(key, "csv"))  { pendingAction = ACT_CSV;   return; }
    else if (!strcmp(key, "r"))    { pendingAction = ACT_RESET; return; }
    else if (!strcmp(key, "de"))   { pendingAction = ACT_DEAD;  return; }
    else if (!strcmp(key, "s"))    { irStream = !irStream;
                                     blePrintf("ir stream %s\n", irStream ? "on" : "off");
                                     return; }
    else if (!strcmp(key, "n")) {
        if (n < 2) { blePrintf("err: n needs value\n"); return; }
        int v = atoi(val);
        if (v < 1)  v = 1;
        if (v > N_SAMP_MAX) v = N_SAMP_MAX;
        nSamples = v;
        blePrintf("ok n=%d\n", nSamples);
        return;
    }
    else if (!strcmp(key, "d")) {
        if (n < 2) { blePrintf("err: d needs cm 1..9\n"); return; }
        int cm = atoi(val);
        if (cm < 1 || cm > N_DIST) {
            blePrintf("err: cm out of range (1..%d)\n", N_DIST);
            return;
        }
        pendingAction = ACT_DIST;
        pendingArg    = cm;
        return;
    }

    blePrintf("err: unknown %s\n", key);
}

// ── BLE callbacks ────────────────────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        bleConnected = true;
        blePrintf("hello neuromouse-cal\n");
        blePrintf("cmds: d <cm>, de, s, ?, csv, r, n <int>\n");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        bleConnected = false;
        NimBLEDevice::startAdvertising();
    }
};
class RXCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        std::string val = c->getValue();
        if (val.empty()) return;
        size_t n = min(val.size(), sizeof(bleCmdBuf) - 1);
        memcpy((void*)bleCmdBuf, val.data(), n);
        ((char*)bleCmdBuf)[n] = 0;
        bleCmdReady = true;
    }
};

static void bleInit() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(NUS_SERVICE_UUID);
    pTX = pService->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* pRX = pService->createCharacteristic(
        NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRX->setCallbacks(new RXCallbacks());
    pService->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName(BLE_DEVICE_NAME);
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->enableScanResponse(true);
    pAdv->start();
}

// ── Button ───────────────────────────────────────────────────────────────────
static bool buttonEdge() {
    static unsigned long pressStart = 0;
    static bool armed = true;
    bool low = (digitalRead(BUTTON_1) == LOW);
    unsigned long now = millis();
    if (!low) { pressStart = 0; armed = true; return false; }
    if (pressStart == 0) pressStart = now;
    if (armed && (now - pressStart >= BUTTON_HOLD_MS)) {
        armed = false;
        return true;
    }
    return false;
}

// ── Menu ─────────────────────────────────────────────────────────────────────
// Items: D1..D9, DeadEnd, Reset, Dump (12 items)
enum { I_D1 = 0, I_DEAD = N_DIST, I_RESET, I_DUMP, I_COUNT };
static int  menuSel    = 0;
static long menuEncRef = 0;

static const char* menuLabel(int idx) {
    static char buf[12];
    if (idx < N_DIST) { snprintf(buf, sizeof(buf), "Cap D%dcm", idx + 1); return buf; }
    if (idx == I_DEAD)  return "Cap DeadEnd";
    if (idx == I_RESET) return "Reset all";
    if (idx == I_DUMP)  return "Dump";
    return "?";
}

static void drawMenu() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    char hdr[28];
    snprintf(hdr, sizeof(hdr), "IR CAL n=%d %s",
             nSamples, bleConnected ? "BLE" : "");
    oled.drawStr(0, 9, hdr);
    oled.drawHLine(0, 11, 128);

    const int VIS = 4;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > I_COUNT - VIS) top = I_COUNT - VIS;
    if (top < 0) top = 0;

    for (int i = 0; i < VIS && (top + i) < I_COUNT; i++) {
        int idx = top + i;
        int y = 14 + i * 10;
        const char* lbl = menuLabel(idx);
        char line[24];
        // Show small "*" marker after captured rows
        bool captured = false;
        if (idx < N_DIST)       captured = distCaps[idx].valid;
        else if (idx == I_DEAD) captured = deadEndCap.valid;
        snprintf(line, sizeof(line), "%s%s", lbl, captured ? " *" : "");

        if (idx == menuSel) {
            oled.drawBox(0, y, 128, 10);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, line);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, line);
        }
    }
    oled.drawHLine(0, 54, 128);
    oled.drawStr(0, 63, statusLine);
    oled.sendBuffer();
}

static void drawCaptureScreen(const char* label, const Capture& c) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    char l1[28], l2[28], l3[28], l4[28], l5[28];
    snprintf(l1, sizeof(l1), "CAP %s  n=%d", label, nSamples);
    snprintf(l2, sizeof(l2), "LF %4d (%d-%d)", c.mean[0], c.vmin[0], c.vmax[0]);
    snprintf(l3, sizeof(l3), "L  %4d (%d-%d)", c.mean[1], c.vmin[1], c.vmax[1]);
    snprintf(l4, sizeof(l4), "R  %4d (%d-%d)", c.mean[2], c.vmin[2], c.vmax[2]);
    snprintf(l5, sizeof(l5), "RF %4d (%d-%d)", c.mean[3], c.vmin[3], c.vmax[3]);
    oled.drawStr(0,  9, l1);
    oled.drawHLine(0, 11, 128);
    oled.drawStr(0, 23, l2);
    oled.drawStr(0, 34, l3);
    oled.drawStr(0, 45, l4);
    oled.drawStr(0, 56, l5);
    oled.sendBuffer();
}

// ── Action dispatch ──────────────────────────────────────────────────────────
static void runDistCapture(int cm) {
    setStatus("cap d%dcm...", cm);
    drawMenu();
    blePrintf("cap d=%dcm start n=%d\n", cm, nSamples);
    Capture& c = distCaps[cm - 1];
    doCapture(c);
    char lbl[6]; snprintf(lbl, sizeof(lbl), "d%dcm", cm);
    dumpRow(lbl, c);
    drawCaptureScreen(lbl, c);
    setStatus("d%d done LF%d", cm, c.mean[0]);
    delay(1200);
}
static void runDeadEndCapture() {
    setStatus("cap deadend...");
    drawMenu();
    blePrintf("cap deade start n=%d\n", nSamples);
    doCapture(deadEndCap);
    dumpRow("deade", deadEndCap);
    drawCaptureScreen("DEAD", deadEndCap);
    setStatus("DE LF%d RF%d", deadEndCap.mean[0], deadEndCap.mean[3]);
    delay(1500);
}

static void dispatchLocal(int sel) {
    if (sel < N_DIST)      runDistCapture(sel + 1);
    else if (sel == I_DEAD) runDeadEndCapture();
    else if (sel == I_RESET){ resetAll();  setStatus("reset"); blePrintf("reset ok\n"); }
    else if (sel == I_DUMP) { dumpAll();   setStatus("dump"); }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(BUTTON_1, INPUT_PULLUP);
    analogReadResolution(12);

    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }

    rightEnc.begin();

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    resetAll();

    bleInit();

    menuEncRef = rightEnc.getTicks();
    setStatus("ready BLE");
    drawMenu();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // 1) drain BLE inbox
    if (bleCmdReady) {
        bleCmdReady = false;
        handleBleCommand((const char*)bleCmdBuf);
    }

    // 2) IR live stream
    static unsigned long lastIrBle = 0;
    if (irStream && bleConnected) {
        unsigned long now = millis();
        if (now - lastIrBle >= IR_STREAM_MS) {
            lastIrBle = now;
            int s[4]; sampleOnce(s);
            blePrintf("ir LF=%d L=%d R=%d RF=%d\n", s[0], s[1], s[2], s[3]);
        }
    }

    // 3) BLE-triggered action
    int act = pendingAction;
    if (act != ACT_NONE) {
        pendingAction = ACT_NONE;
        switch (act) {
            case ACT_DUMP:  dumpAll(); setStatus("dump"); break;
            case ACT_CSV:   dumpCsv(); setStatus("csv");  break;
            case ACT_RESET: resetAll(); setStatus("reset"); blePrintf("reset ok\n"); break;
            case ACT_DEAD:  runDeadEndCapture(); break;
            case ACT_DIST:  runDistCapture(pendingArg); break;
            default: break;
        }
        drawMenu();
        return;
    }

    // 4) Menu scroll via right encoder wheel
    long delta = rightEnc.getTicks() - menuEncRef;
    if (delta >= ENC_PER_STEP) {
        menuSel = (menuSel + 1) % I_COUNT;
        menuEncRef += ENC_PER_STEP;
        drawMenu();
        return;
    }
    if (delta <= -ENC_PER_STEP) {
        menuSel = (menuSel - 1 + I_COUNT) % I_COUNT;
        menuEncRef -= ENC_PER_STEP;
        drawMenu();
        return;
    }

    if (buttonEdge()) {
        dispatchLocal(menuSel);
        menuEncRef = rightEnc.getTicks();
        drawMenu();
    }

    // 5) periodic redraw so header BLE flag stays current
    static unsigned long lastDraw = 0;
    unsigned long now = millis();
    if (now - lastDraw >= OLED_REDRAW_MS * 5) {
        lastDraw = now;
        drawMenu();
    }
}
