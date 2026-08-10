# test/ — standalone PlatformIO sketches

Each sketch maps to an `[env:…]` in `platformio.ini`. Build with:

```bash
pio run -e <env-name>
```

| Env | Source | Notes |
|---|---|---|
| `main` | `src/main.cpp` | Production firmware |
| `sensor-cal` | `sensor_cal.cpp` | IR / sensor calibration |
| `encoder-test` | `encoder-test.cpp` | PCNT tick / RPM |
| `imu-turn` | `mpu6500.cpp` | Gyro bias + yaw stream |
| `motor-freq` | `motor-freq-config.cpp` | PWM frequency experiments |
| `pivot-turn` | `pivot-turn.cpp` | Pivot / spot turn bench |
| `ir-turn-test` | `ir-turn-test.cpp` | IR-assisted turn tests |
| `wall-follow-pcnt` | `wall-follow-encoder-pcnt.cpp` | Wall-follow reference (PCNT) |
| `goal-blink` | `goal-blink.cpp` | Goal LED celebration |
| `ws2812b` | `ws2812b.cpp` | Onboard WS2812 smoke test |
| `batt-volt` | `batt-volt.cpp` | Battery divider readout |
| `ble-test` | `ble-test.cpp` | Nordic UART BLE smoke test |
