# IMU (MPU-6500) Complete Test

## main.cpp

```cpp
// main.cpp — MPU-6500 IMU complete test (minimal)
// Tests: I2C ping, WHO_AM_I, all 6 axes (accel + gyro), temp, bias calibration, yaw integration
#include <Arduino.h>
#include <Wire.h>
#include "PinConfig.h"

// MPU-6500 register map
#define MPU_ADDR        0x68
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT_H 0x3B  // accel X,Y,Z + temp + gyro X,Y,Z = 14 bytes total
#define REG_TEMP_OUT_H  0x41
#define REG_GYRO_XOUT_H 0x43

// Scales
#define ACCEL_SCALE     16384.0f  // LSB/g  at ±2g (AFS_SEL=0)
#define GYRO_SCALE      131.0f    // LSB/(°/s) at ±250°/s (FS_SEL=0)
#define TEMP_OFFSET     0.0f      // °C offset (MPU-6500: Temp = raw/333.87 + 21.0)

// Bias calibration samples
#define CALIB_SAMPLES   200

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0;
float yaw = 0, pitch = 0, roll = 0;
unsigned long lastUpdate = 0;

bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
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

int16_t to16(uint8_t hi, uint8_t lo) {
    return (int16_t)((hi << 8) | lo);
}

struct RawData { int16_t ax, ay, az, temp, gx, gy, gz; };
bool readAll(RawData& d) {
    uint8_t buf[14];
    if (!readRegs(REG_ACCEL_XOUT_H, buf, 14)) return false;
    d.ax   = to16(buf[0],  buf[1]);
    d.ay   = to16(buf[2],  buf[3]);
    d.az   = to16(buf[4],  buf[5]);
    d.temp = to16(buf[6],  buf[7]);
    d.gx   = to16(buf[8],  buf[9]);
    d.gy   = to16(buf[10], buf[11]);
    d.gz   = to16(buf[12], buf[13]);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[IMU-TEST] ==============================");
    Serial.println("[IMU-TEST] MPU-6500 complete test");
    Serial.println("[IMU-TEST] ==============================\n");

    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);

    // STEP 1: I2C ping
    Serial.printf("[1] I2C ping addr=0x%02X ... ", MPU_ADDR);
    Wire.beginTransmission(MPU_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("FAIL (error %d) — check SDA=GPIO%d SCL=GPIO%d\n", err, IMU_SDA, IMU_SCL);
        while (1) delay(1000);
    }
    Serial.println("OK");

    // STEP 2: WHO_AM_I
    uint8_t who = 0;
    readRegs(REG_WHO_AM_I, &who, 1);
    Serial.printf("[2] WHO_AM_I = 0x%02X  expect=0x70  → %s\n",
                  who, who == 0x70 ? "PASS" : "FAIL (wrong device?)");

    // STEP 3: Wake up
    Serial.print("[3] Wake up (PWR_MGMT_1=0x00) ... ");
    Serial.println(writeReg(REG_PWR_MGMT_1, 0x00) ? "OK" : "FAIL");
    delay(100);

    // STEP 4: Config
    Serial.print("[4] Gyro  FS_SEL=0 (±250°/s)  ... ");
    Serial.println(writeReg(REG_GYRO_CFG, 0x00) ? "OK" : "FAIL");
    Serial.print("[4] Accel AFS_SEL=0 (±2g)      ... ");
    Serial.println(writeReg(REG_ACCEL_CFG, 0x00) ? "OK" : "FAIL");

    // STEP 5: Single raw read
    Serial.println("\n[5] Single raw read:");
    RawData d;
    if (!readAll(d)) {
        Serial.println("    FAIL — could not read sensor registers");
    } else {
        float ax_g  = d.ax  / ACCEL_SCALE;
        float ay_g  = d.ay  / ACCEL_SCALE;
        float az_g  = d.az  / ACCEL_SCALE;
        float gx_ds = d.gx  / GYRO_SCALE;
        float gy_ds = d.gy  / GYRO_SCALE;
        float gz_ds = d.gz  / GYRO_SCALE;
        float tempC = d.temp / 333.87f + 21.0f;
        Serial.printf("    Accel : X=%.3fg  Y=%.3fg  Z=%.3fg  (expect ~0,0,+1 when flat)\n", ax_g, ay_g, az_g);
        Serial.printf("    Gyro  : X=%.3f°/s  Y=%.3f°/s  Z=%.3f°/s  (expect ~0 when still)\n", gx_ds, gy_ds, gz_ds);
        Serial.printf("    Temp  : %.2f°C\n", tempC);
    }

    // STEP 6: Bias calibration
    Serial.printf("\n[6] Bias calibration (%d samples — keep robot STILL) ...\n", CALIB_SAMPLES);
    float sgx=0, sgy=0, sgz=0, sax=0, say=0;
    int good = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        RawData s;
        if (readAll(s)) {
            sgx += s.gx / GYRO_SCALE;
            sgy += s.gy / GYRO_SCALE;
            sgz += s.gz / GYRO_SCALE;
            sax += s.ax / ACCEL_SCALE;
            say += s.ay / ACCEL_SCALE;
            good++;
        }
        if (i % 50 == 0) Serial.printf("    sample %d/%d\n", i, CALIB_SAMPLES);
        delay(2);
    }
    gyroBiasX  = sgx / good;
    gyroBiasY  = sgy / good;
    gyroBiasZ  = sgz / good;
    accelBiasX = sax / good;
    accelBiasY = say / good;
    Serial.printf("    Gyro  bias: X=%.4f  Y=%.4f  Z=%.4f  °/s\n", gyroBiasX, gyroBiasY, gyroBiasZ);
    Serial.printf("    Accel bias: X=%.4f  Y=%.4f  g\n", accelBiasX, accelBiasY);
    Serial.printf("    Good reads: %d/%d\n", good, CALIB_SAMPLES);

    if (fabsf(gyroBiasZ) > 1.0f)
        Serial.println("    WARN: gyro Z bias > 1°/s — was robot moving during calibration?");
    else
        Serial.println("    Gyro bias OK");

    lastUpdate = micros();

    Serial.println("\n[IMU-TEST] Live stream starting (20 Hz)");
    Serial.println("[IMU-TEST] Format: yaw pitch roll | ax ay az | gx gy gz | temp");
    Serial.println("[IMU-TEST] ==============================\n");
}

void loop() {
    RawData d;
    if (!readAll(d)) {
        Serial.println("[IMU-TEST] read FAIL");
        delay(100);
        return;
    }

    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1e6f;
    lastUpdate = now;

    float ax = d.ax / ACCEL_SCALE - accelBiasX;
    float ay = d.ay / ACCEL_SCALE - accelBiasY;
    float az = d.az / ACCEL_SCALE;
    float gx = d.gx / GYRO_SCALE  - gyroBiasX;
    float gy = d.gy / GYRO_SCALE  - gyroBiasY;
    float gz = d.gz / GYRO_SCALE  - gyroBiasZ;
    float tempC = d.temp / 333.87f + 21.0f;

    if (fabsf(gx) < 0.05f) gx = 0;
    if (fabsf(gy) < 0.05f) gy = 0;
    if (fabsf(gz) < 0.05f) gz = 0;

    yaw   += gz * dt;
    pitch += gy * dt;
    roll  += gx * dt;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();
        Serial.printf("yaw=%7.2f°  pitch=%7.2f°  roll=%7.2f° | "
                      "ax=%6.3fg ay=%6.3fg az=%6.3fg | "
                      "gx=%6.2f gy=%6.2f gz=%6.2f °/s | "
                      "%.1f°C\n",
                      yaw, pitch, roll,
                      ax, ay, az,
                      gx, gy, gz,
                      tempC);
    }
}
```

## Test steps

| Step | What | Pass condition |
|------|------|----------------|
| 1 | I2C ping | No error — halts if device missing |
| 2 | WHO_AM_I | `0x70` |
| 3 | Wake from sleep | Write OK |
| 4 | Config gyro + accel | Write OK |
| 5 | Single raw read | `az ≈ +1.0g`, `ax/ay ≈ 0`, `temp 25–40°C` |
| 6 | Bias calibration | `\|gyroBiasZ\| < 1.0°/s` |
| Live | 20 Hz stream | yaw/pitch/roll drift slowly only when still |

## Notes
- Robot must be completely still during step 6
- Gyro integrates drift — yaw will wander slowly over minutes, that is normal
- `az ≈ +1g` confirms accel Z axis sees gravity correctly
