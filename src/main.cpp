// main.cpp — IR + WS2812B (8 LEDs) + IMU + Battery + BLE
//
// LED map:
//   0 = LF      1 = L45     2,3 = unused (black)
//   4 = R45     5 = RF
//   6 = battery  blue(full) → red(low)   range 6.8–8.4V
//   7 = yaw      hue sweeps 0–360° as robot rotates, brightness = |rate|
//
// BLE: connect "Micromouse26" in Serial Bluetooth Terminal
#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include "PinConfig.h"

// ── WS2812B ──────────────────────────────────────────────────────────────────
#define NUM_LEDS      8
#define LED_BRIGHT    5 // always keep it 5 percent or below to avoid blinding and excessive power draw
CRGB leds[NUM_LEDS];

// IR delta 0–4000 → green→red
CRGB deltaToColor(int delta) {
    uint8_t r = map(constrain(delta, 0, 4000), 0, 4000, 0,   255);
    uint8_t g = map(constrain(delta, 0, 4000), 0, 4000, 255,   0);
    return CRGB(r, g, 0);
}

// Battery 6.8–8.4V → blue(full)→red(low)
CRGB battToColor(float v) {
    float pct = constrain((v - 6.8f) / (8.4f - 6.8f), 0.0f, 1.0f); // 0=empty 1=full
    uint8_t r = map((int)(pct * 100), 0, 100, 255, 0);
    uint8_t b = map((int)(pct * 100), 0, 100, 0,   255);
    return CRGB(r, 0, b);
}

// Yaw → hue (0–255 wraps full circle), brightness scales with rotation rate
CRGB yawToColor(float yawDeg, float rateDps) {
    uint8_t hue   = (uint8_t)(fmod(yawDeg + 360.0f, 360.0f) * 255.0f / 360.0f);
    uint8_t bright = (uint8_t)constrain(fabsf(rateDps) * 2.0f, 20.0f, 255.0f);
    return CHSV(hue, 255, bright);
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
    { "LF ", EMIT_LF,  RX_LF,  0 },
    { "L45", EMIT_L45, RX_L45, 1 },
    { "R45", EMIT_R45, RX_R45, 4 },
    { "RF ", EMIT_RF,  RX_RF,  5 },
};
static const int N_IR = 4;

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

// ── IMU (MPU-6500) ────────────────────────────────────────────────────────────
#define MPU_ADDR  0x68

float gyroBiasZ = 0.0f;
float yawDeg    = 0.0f;
float gyroRate  = 0.0f;
unsigned long lastIMU = 0;

void imuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

float readGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    return raw / 131.0f;
}

void imuBegin() {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    imuWrite(0x6B, 0x00);  // wake
    delay(100);
    // calibrate bias — 200 samples
    float sum = 0;
    for (int i = 0; i < 200; i++) { sum += readGyroZ(); delay(2); }
    gyroBiasZ = sum / 200.0f;
    lastIMU = micros();
    blePrintf("[IMU] bias=%.4f dps\n", gyroBiasZ);
}

void imuUpdate() {
    unsigned long now = micros();
    float dt = (now - lastIMU) / 1e6f;
    lastIMU = now;
    float rate = readGyroZ() - gyroBiasZ;
    if (fabsf(rate) < 0.05f) rate = 0;
    gyroRate = rate;
    yawDeg  += rate * dt;
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    // IR pins
    for (int i = 0; i < N_IR; i++) {
        pinMode(PAIRS[i].emit, OUTPUT);
        digitalWrite(PAIRS[i].emit, LOW);
        pinMode(PAIRS[i].rx, INPUT);
    }

    // LEDs
    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHT);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    bleSetup();
    imuBegin();

    blePrintf("\n[INIT] 8-LED test ready\n");
    blePrintf("LED0=LF LED1=L45 LED4=R45 LED5=RF LED6=batt LED7=yaw\n\n");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    imuUpdate();

    // IR — LEDs 0,1,4,5
    int d[N_IR];
    for (int i = 0; i < N_IR; i++) {
        d[i] = readDelta(PAIRS[i]);
        leds[PAIRS[i].ledIdx] = deltaToColor(d[i]);
    }

    // LEDs 2,3 unused
    leds[2] = CRGB::Black;
    leds[3] = CRGB::Black;

    // LED 6 — battery
    int   batRaw = analogRead(BAT_V_SENSE);
    float vBat   = (batRaw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
    leds[6] = battToColor(vBat);

    // LED 7 — yaw
    leds[7] = yawToColor(yawDeg, gyroRate);

    FastLED.show();

    // BLE output every 200ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        blePrintf("LF:%4d %s|L45:%4d %s|R45:%4d %s|RF:%4d %s|%.2fV|yaw:%.1f\n",
                  d[0], classify(d[0]),
                  d[1], classify(d[1]),
                  d[2], classify(d[2]),
                  d[3], classify(d[3]),
                  vBat, yawDeg);
    }
}
