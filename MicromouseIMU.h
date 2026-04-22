#pragma once
#include <Wire.h>

class MicromouseIMU {
private:
    const int MPU_ADDR = 0x68;
    float gyroBiasZ = 0;
    float currentYaw = 0;
    unsigned long lastUpdate = 0;

public:
    void begin(int sda, int scl) {
        Wire.begin(sda, scl);
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B); // PWR_MGMT_1
        Wire.write(0);    // Wake up
        Wire.endTransmission();
        calibrate();
    }

    void calibrate() {
        float sum = 0;
        for(int i=0; i<200; i++) {
            sum += readRawZ();
            delay(2);
        }
        gyroBiasZ = sum / 200.0;
    }

    float readRawZ() {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);
        int16_t raw = (Wire.read() << 8) | Wire.read();
        return (float)raw / 131.0; // FS_SEL=0 (250 deg/s)
    }

    void update() {
        unsigned long now = micros();
        float dt = (now - lastUpdate) / 1000000.0;
        lastUpdate = now;
        float rate = readRawZ() - gyroBiasZ;
        if (abs(rate) > 0.05) currentYaw += rate * dt; // Simple noise floor
    }

    float getYaw() { return currentYaw; }
    void resetYaw() { currentYaw = 0; }
};