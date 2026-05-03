// main.cpp — IR sensor test + WS2812B visual indicator + BLE
// LED mapping (0-indexed):
//   LED 0 = LF    LED 1 = L45    LED 2,3 = unused    LED 4 = R45    LED 5 = RF
// Color: green (delta=0) → yellow → orange → red (delta>=4000)
// BLE: connect "Micromouse26" in Serial Bluetooth Terminal
#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"

// ── WS2812B ──────────────────────────────────────────────────────────────────
#define NUM_LEDS  6
#define LED_BRIGHTNESS 5
CRGB leds[NUM_LEDS];

// Map delta 0–4000 → green→red
CRGB deltaToColor(int delta) {
    int clamped = constrain(delta, 0, 4000);
    uint8_t r = map(clamped, 0, 4000, 0,   255);
    uint8_t g = map(clamped, 0, 4000, 255, 0);
    return CRGB(r, g, 0);
}

// ── BLE NUS ──────────────────────────────────────────────────────────────────
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
}

// ── IR ────────────────────────────────────────────────────────────────────────
struct IRPair { const char* name; uint8_t emit; uint8_t rx; uint8_t ledIdx; };
static const IRPair PAIRS[] = {
    { "LF ", EMIT_LF,  RX_LF,  0 },  // LED 0
    { "L45", EMIT_L45, RX_L45, 1 },  // LED 1
    { "R45", EMIT_R45, RX_R45, 4 },  // LED 4
    { "RF ", EMIT_RF,  RX_RF,  5 },  // LED 5
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

const char* classify(int d) {
    if (d < 50)  return "open";
    if (d < 300) return "far";
    if (d < 800) return "near";
    return "WALL";
}

// ── setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    // IR pins
    for (int i = 0; i < N; i++) {
        pinMode(PAIRS[i].emit, OUTPUT);
        digitalWrite(PAIRS[i].emit, LOW);
        pinMode(PAIRS[i].rx, INPUT);
    }

    // LEDs — init all off, unused LEDs 2,3 stay black
    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    bleSetup();

    blePrintf("\n[IR+LED] LF=LED0 L45=LED1 R45=LED4 RF=LED5\n");
    blePrintf("[IR+LED] green=0 -> red=4000\n\n");
}

// ── loop ─────────────────────────────────────────────────────────────────────
void loop() {
    int d[N];
    for (int i = 0; i < N; i++) {
        d[i] = readDelta(PAIRS[i]);
        leds[PAIRS[i].ledIdx] = deltaToColor(d[i]);
    }
    FastLED.show();

    blePrintf("LF:%4d %s | L45:%4d %s | R45:%4d %s | RF:%4d %s\n",
              d[0], classify(d[0]),
              d[1], classify(d[1]),
              d[2], classify(d[2]),
              d[3], classify(d[3]));
    delay(200);
}
