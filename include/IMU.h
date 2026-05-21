// include/IMU.h — MPU-6500 gyro + yaw integration.
//
// Single-axis (Z) gyro read with auto bias capture and a low-pass filtered
// derivative for the PID. `updateYaw()` should be called every loop iteration
// so the integration uses fresh dt.
//
// Globals (read by motion executor, written here):
//   yawDeg          — integrated heading angle, ° (resets each phaseEnter)
//   gzFilt          — low-pass yaw rate, °/s
//   yawTargetDeg    — phaseEnter sets this from runTarget × dir
//   phaseStartYawDeg — yaw snapshot at phase entry (currently always 0)
//   imuReady        — true after mpuInit()
//
// Convention: right rotation → yawDeg < 0.

#ifndef MM26_IMU_H
#define MM26_IMU_H

#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR        0x68
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define GYRO_FS_SEL     0x10
#define GYRO_SCALE      32.8f
#define DLPF_CFG_VAL    0x03

static float    gyroBiasZ        = 0.0f;
static float    yawDeg           = 0.0f;
static float    gzFilt           = 0.0f;
static uint32_t lastYawUs        = 0;
static bool     imuReady         = false;
static float    yawTargetDeg     = 0.0f;
static float    phaseStartYawDeg = 0.0f;

static bool mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool mpuReadN(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}

static bool readGzRaw(int16_t& out) {
    uint8_t b[2];
    if (!mpuReadN(0x47, b, 2)) return false;
    out = (int16_t)((b[0] << 8) | b[1]);
    return true;
}

static bool mpuInit() {
    uint8_t who = 0;
    if (!mpuReadN(REG_WHO_AM_I, &who, 1)) return false;
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_CONFIG,     DLPF_CFG_VAL);
    mpuWrite(REG_GYRO_CFG,   GYRO_FS_SEL);
    mpuWrite(REG_ACCEL_CFG,  0x00);
    delay(20);
    return true;
}

static void calibrateGyroBias(int N = 300, int sampleDelayMs = 2) {
    float sum = 0; int good = 0;
    for (int i = 0; i < N; i++) {
        int16_t raw;
        if (readGzRaw(raw)) { sum += raw / GYRO_SCALE; good++; }
        delay(sampleDelayMs);
    }
    if (good > 0) gyroBiasZ = sum / good;
    gzFilt    = 0.0f;
    lastYawUs = micros();
}

static void updateYaw() {
    if (!imuReady) return;
    int16_t raw;
    if (!readGzRaw(raw)) return;
    uint32_t now = micros();
    float dt = (lastYawUs == 0) ? 0.001f : (now - lastYawUs) / 1.0e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastYawUs = now;
    float gz = raw / GYRO_SCALE - gyroBiasZ;
    if (fabsf(gz) < 0.05f) gz = 0.0f;
    gzFilt = 0.7f * gzFilt + 0.3f * gz;
    yawDeg += gz * dt;
}

#endif
