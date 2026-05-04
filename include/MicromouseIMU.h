// =============================================================================
// MicromouseIMU.h
// MPU-6500 (I2C) gyroscope driver — yaw integration only.
//
// MPU-6500 register map (abbreviated):
//   0x6B  PWR_MGMT_1  — wake up: write 0x00
//   0x1B  GYRO_CONFIG — full-scale range; default FS_SEL=0 = ±250 °/s
//   0x47  GYRO_ZOUT_H — Z-axis gyro high byte (Z = yaw for flat-mounted IMU)
//   0x48  GYRO_ZOUT_L — Z-axis gyro low  byte
//
// Sensitivity at FS_SEL=0: 131 LSB per °/s → divide raw by 131 → °/s
//
// Calibration:
//   Reads 200 samples at startup (robot must be perfectly still!).
//   Average = gyroBiasZ. Subtracted from every subsequent reading.
//
// Noise floor:
//   Readings whose absolute value < 0.05 °/s are treated as zero to prevent
//   slow yaw drift when the robot is stationary.
// =============================================================================
#pragma once
#include <Wire.h>

class MicromouseIMU {
private:
    const int   MPU_ADDR    = 0x68;
    const float GYRO_SCALE  = 131.0f;  // LSB/(°/s) at FS_SEL=0
    const float NOISE_FLOOR = 0.05f;   // °/s — rates below this are ignored

    float         gyroBiasZ  = 0.0f;
    float         currentYaw = 0.0f;
    unsigned long lastUpdate  = 0;

public:
    // --------------------------------------------------------------------------
    // begin() — wake up IMU, configure, then calibrate bias
    // --------------------------------------------------------------------------
    void begin(int sda, int scl) {
        Serial.printf("[IMU] begin() — SDA=GPIO%d  SCL=GPIO%d  addr=0x%02X\n", sda, scl, MPU_ADDR);
        Wire.begin(sda, scl);

        // Wake up: clear sleep bit in PWR_MGMT_1
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B);
        Wire.write(0x00);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.println(F("[IMU] PWR_MGMT_1 write OK — MPU-6500 awake"));
        } else {
            Serial.printf("[IMU] ERROR: PWR_MGMT_1 write failed (I2C error %d) — check wiring!\n", err);
        }

        // Short settle time before calibration
        delay(100);
        Serial.println(F("[IMU] begin() — starting gyro bias calibration (keep robot still!)"));
        calibrate();

        lastUpdate = micros();
        Serial.printf("[IMU] begin() done — gyroBiasZ=%.4f °/s\n", gyroBiasZ);
    }

    // --------------------------------------------------------------------------
    // calibrate() — collect 200 samples to estimate Z-axis bias
    // --------------------------------------------------------------------------
    void calibrate() {
        Serial.println(F("[IMU] calibrate() — collecting 200 gyro Z samples over ~400 ms"));
        float sum  = 0.0f;
        int   good = 0;

        for (int i = 0; i < 200; i++) {
            float z = readRawZ();
            sum += z;
            good++;
            if (i % 50 == 0) {
                Serial.printf("[IMU] calibrate() sample %3d: rawZ=%.3f °/s\n", i, z);
            }
            delay(2);
        }
        gyroBiasZ = sum / (float)good;
        Serial.printf("[IMU] calibrate() done — gyroBiasZ=%.4f °/s (from %d samples)\n",
                      gyroBiasZ, good);
    }

    // --------------------------------------------------------------------------
    // readRawZ() — read GYRO_ZOUT registers and convert to °/s (un-calibrated)
    // --------------------------------------------------------------------------
    float readRawZ() {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47);  // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);
        int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
        return (float)raw / GYRO_SCALE;
    }

    // --------------------------------------------------------------------------
    // update() — must be called every loop iteration
    // Integrates calibrated gyro Z rate → yaw angle
    // --------------------------------------------------------------------------
    void update() {
        unsigned long now = micros();
        float dt = (float)(now - lastUpdate) / 1000000.0f;
        lastUpdate = now;

        float rate = readRawZ() - gyroBiasZ;

        // Apply noise floor: ignore tiny rates that are just bias residue
        if (fabsf(rate) < NOISE_FLOOR) {
            // Below noise floor — skip integration
            return;
        }

        float delta = rate * dt;
        currentYaw += delta;
    }

    // --------------------------------------------------------------------------
    // getYaw() — current integrated yaw angle in degrees
    // --------------------------------------------------------------------------
    float getYaw() const { return currentYaw; }

    // --------------------------------------------------------------------------
    // getBias() — calibrated Z-axis gyro bias in °/s (used by self-test)
    // --------------------------------------------------------------------------
    float getBias() const { return gyroBiasZ; }

    // --------------------------------------------------------------------------
    // resetYaw() — zero the accumulated yaw (call at start of each move)
    // --------------------------------------------------------------------------
    void resetYaw() {
        Serial.printf("[IMU] resetYaw() — was %.3f°, now 0.0°\n", currentYaw);
        currentYaw = 0.0f;
    }

    // --------------------------------------------------------------------------
    // printStatus() — snapshot of IMU state
    // --------------------------------------------------------------------------
    void printStatus() {
        Serial.printf("[IMU] === Status: yaw=%.3f°  bias=%.4f°/s  noiseFloor=%.3f°/s ===\n",
                      currentYaw, gyroBiasZ, NOISE_FLOOR);
    }
};
