// main.cpp — IR + WS2812B (8 LEDs) + IMU + Battery + BLE + Motor + Encoder
//
// LED map:
//   0 = LF      1 = L45     2,3 = unused (black)
//   4 = R45     5 = RF
//   6 = battery  blue(full) → red(low)   range 6.8–8.4V
//   7 = yaw      hue sweeps 0–360° as robot rotates, brightness = |rate|
//
// Button: press → move forward one cell (180 mm) using encoder feedback
// BLE: connect "Micromouse26" in Serial Bluetooth Terminal
#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Motor + Encoder ───────────────────────────────────────────────────────────
// Derived from PinConfig.h constants:
//   circumference = π × WHEEL_DIAMETER = π × 32 = 100.53 mm
//   mm per tick   = circumference / TICKS_PER_REV = 100.53 / 210 ≈ 0.4787 mm
//   ticks per cell = CELL_MM / mm_per_tick = 180 / 0.4787 ≈ 376 ticks
#define CELL_MM         180.0f
#define MM_PER_TICK     ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)  // ~0.4787
#define TICKS_PER_CELL  ((long)(CELL_MM / MM_PER_TICK))                   // ~376

// Open-loop drive PWM for forward motion (tune if robot drifts badly)
// Start low — raise until it moves reliably without stalling
#define DRIVE_PWM       400  // range 0–1023; ~40% duty

MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "LEFT");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "RIGHT");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B, "LEFT");
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B, "RIGHT");

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt();  }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

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

// Yaw → hue (0–255 wraps full circle), brightness scales with rotation rate.
// Off when stationary (rate near zero), brightens as robot rotates.
CRGB yawToColor(float yawDeg, float rateDps) {
    uint8_t hue    = (uint8_t)(fmod(yawDeg + 360.0f, 360.0f) * 255.0f / 360.0f);
    uint8_t bright = (uint8_t)constrain(fabsf(rateDps) * 2.0f, 0.0f, 255.0f);
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
    return max(0, lit - ambient);  // clamp: negative = noise, treat as 0
}

const char* classify(int d) {
    if (d < 50)  return "open";
    if (d < 300) return "far";
    if (d < 800) return "near";
    return "WALL";
}

// ── IMU (MPU-6500) ────────────────────────────────────────────────────────────
// Register map (abbreviated):
//   0x3B  ACCEL_XOUT_H  — start of 14-byte burst: AX AY AZ TEMP GX GY GZ
//   0x6B  PWR_MGMT_1    — write 0x00 to wake
//   0x75  WHO_AM_I      — should read 0x70
// Gyro scale at FS_SEL=0: 131 LSB/(°/s)
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define GYRO_SCALE       131.0f
#define GYRO_NOISE_FLOOR 0.05f   // °/s — rates below this treated as zero
#define CALIB_SAMPLES    200

float         gyroBiasZ = 0.0f;
float         yawDeg    = 0.0f;
float         gyroRate  = 0.0f;
unsigned long lastIMU   = 0;

// Write one register; returns true on success
bool imuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Burst-read 14 bytes from 0x3B: AX(2) AY(2) AZ(2) TEMP(2) GX(2) GY(2) GZ(2)
// Returns false if I2C fails. Only GZ used for yaw but full burst is cheaper
// than a targeted 2-byte read (one transaction vs two).
struct IMURaw { int16_t ax, ay, az, temp, gx, gy, gz; };
bool imuReadAll(IMURaw& d) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return false;
    uint8_t b[14];
    for (int i = 0; i < 14; i++) b[i] = Wire.read();
    d.ax   = (int16_t)((b[0]  << 8) | b[1]);
    d.ay   = (int16_t)((b[2]  << 8) | b[3]);
    d.az   = (int16_t)((b[4]  << 8) | b[5]);
    d.temp = (int16_t)((b[6]  << 8) | b[7]);
    d.gx   = (int16_t)((b[8]  << 8) | b[9]);
    d.gy   = (int16_t)((b[10] << 8) | b[11]);
    d.gz   = (int16_t)((b[12] << 8) | b[13]);
    return true;
}

void imuBegin() {
    // begin() then setClock() — setClock() before begin() has no effect
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);  // 400 kHz fast mode

    if (!imuWrite(REG_PWR_MGMT_1, 0x00)) {
        blePrintf("[IMU] ERROR: wake write failed — check SDA=GPIO%d SCL=GPIO%d\n",
                  IMU_SDA, IMU_SCL);
    }
    delay(100);  // let gyro stabilise after wake

    // Calibrate Z bias — count only successful reads
    blePrintf("[IMU] calibrating (%d samples, keep robot still)...\n", CALIB_SAMPLES);
    float sum = 0.0f;
    int   good = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        IMURaw d;
        if (imuReadAll(d)) {
            sum += d.gz / GYRO_SCALE;
            good++;
        }
        delay(2);
    }
    if (good == 0) {
        blePrintf("[IMU] ERROR: 0 good reads during calibration!\n");
    } else {
        gyroBiasZ = sum / good;
        blePrintf("[IMU] bias=%.4f dps  (from %d/%d good reads)\n",
                  gyroBiasZ, good, CALIB_SAMPLES);
        if (fabsf(gyroBiasZ) > 1.0f)
            blePrintf("[IMU] WARN: bias > 1 dps — was robot moving?\n");
    }
    lastIMU = micros();
}

void imuUpdate() {
    unsigned long now = micros();
    float dt = (now - lastIMU) / 1e6f;
    lastIMU = now;

    IMURaw d;
    if (!imuReadAll(d)) return;  // skip update on I2C failure

    float rate = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(rate) < GYRO_NOISE_FLOOR) rate = 0.0f;
    gyroRate = rate;
    yawDeg  += rate * dt;
}

// ── Motion ────────────────────────────────────────────────────────────────────
// moveForwardOneCell() — drives forward until BOTH encoders reach TICKS_PER_CELL.
//
// Open-loop: both motors run at DRIVE_PWM. No PID yet.
// The encoder is the STOP condition — robot halts exactly at tick target.
//
// Ramp-down: at 80% of target, drop to half power to reduce overshoot.
// Safety timeout: 5 seconds — brakes and reports if target never reached.
//
// LEDs 2+3 show progress: black→green as ticks accumulate.
void moveForwardOneCell() {
    leftEnc.reset();
    rightEnc.reset();

    blePrintf("[MOVE] start — target=%ld ticks (%.1f mm)\n",
              TICKS_PER_CELL, (float)TICKS_PER_CELL * MM_PER_TICK);

    leftMotor.drive(DRIVE_PWM);
    rightMotor.drive(DRIVE_PWM);

    unsigned long startMs  = millis();
    bool          ramped   = false;
    const long    rampAt   = (long)(TICKS_PER_CELL * 0.80f);  // 80% → half power

    while (true) {
        long tL = leftEnc.getTicks();
        long tR = rightEnc.getTicks();
        long avg = (tL + tR) / 2;

        // Ramp down at 80% to reduce overshoot
        if (!ramped && avg >= rampAt) {
            ramped = true;
            leftMotor.drive(DRIVE_PWM / 2);
            rightMotor.drive(DRIVE_PWM / 2);
            blePrintf("[MOVE] ramp-down at tick %ld\n", avg);
        }

        // Progress on LEDs 2+3 (green intensity = fraction of cell done)
        uint8_t prog = (uint8_t)constrain(avg * 255 / TICKS_PER_CELL, 0, 255);
        leds[2] = CRGB(0, prog, 0);
        leds[3] = CRGB(0, prog, 0);
        FastLED.show();

        // Stop when both encoders hit target
        if (tL >= TICKS_PER_CELL && tR >= TICKS_PER_CELL) {
            leftMotor.brake();
            rightMotor.brake();
            delay(60);
            leftMotor.coast();
            rightMotor.coast();
            leds[2] = CRGB::Green;
            leds[3] = CRGB::Green;
            FastLED.show();
            float distL = tL * MM_PER_TICK;
            float distR = tR * MM_PER_TICK;
            blePrintf("[MOVE] done — L=%ld ticks (%.1fmm)  R=%ld ticks (%.1fmm)  t=%lums\n",
                      tL, distL, tR, distR, millis() - startMs);
            return;
        }

        // Safety timeout — motors off, report error
        if (millis() - startMs > 5000) {
            leftMotor.brake();
            rightMotor.brake();
            delay(60);
            leftMotor.coast();
            rightMotor.coast();
            leds[2] = CRGB::Red;
            leds[3] = CRGB::Red;
            FastLED.show();
            blePrintf("[MOVE] TIMEOUT — L=%ld  R=%ld  target=%ld — check motors/encoders\n",
                      leftEnc.getTicks(), rightEnc.getTicks(), TICKS_PER_CELL);
            return;
        }
    }
}

// ── Buzzer helpers ────────────────────────────────────────────────────────────
void beep(int ms) {
    ledcWrite(BUZZER_LEDC_CH, BUZZER_DUTY);
    delay(ms);
    ledcWrite(BUZZER_LEDC_CH, 0);
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    // Buzzer — init before anything else so startup beep confirms MCU alive
    ledcSetup(BUZZER_LEDC_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);
    beep(80);  // short startup beep — hardware alive

    // Button
    pinMode(BUTTON_1, INPUT_PULLUP);

    // DRV8833 wake — must be HIGH before motor commands
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    // Motors
    leftMotor.begin();
    rightMotor.begin();

    // Encoders
    leftEnc.begin(leftISR);
    rightEnc.begin(rightISR);

    blePrintf("[INIT] TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f  CELL_MM=%.0f\n",
              TICKS_PER_CELL, MM_PER_TICK, CELL_MM);

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

    beep(40); delay(60); beep(40);  // double beep = init complete
    blePrintf("\n[INIT] ready — LED0=LF LED1=L45 LED4=R45 LED5=RF LED6=batt LED7=yaw\n\n");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // Button: falling edge → move one cell
    static bool lastBtn = HIGH;
    bool btn = digitalRead(BUTTON_1);
    if (lastBtn == HIGH && btn == LOW) {
        blePrintf("[BTN] pressed → moveForwardOneCell()\n");
        beep(60);
        moveForwardOneCell();
        beep(60);  // done beep
    }
    lastBtn = btn;

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
