## Summary
<!-- What changed and why -->

## Type of change
- [ ] Bug fix
- [ ] Tuning / constants (describe chassis if changed)
- [ ] Docs
- [ ] New test sketch (`test/` + matching `platformio.ini` env)
- [ ] Hardware port (link related issue)

## Checklist
- [ ] `pio run -e main` builds cleanly
- [ ] No FreeRTOS / `WifiDebug.h` pulled into `[env:main]`
- [ ] Pins only in `PinConfig.h`; tuning only in `Tuning.h`
- [ ] LEDC remains Arduino 2.x (`ledcSetup` + `ledcAttachPin`)
- [ ] If `CELL_TICKS` / `RIGHT_ENC_SCALE` / IR cal changed: hardware described below
- [ ] Serial verification @ 115200 (or N/A for docs-only)
- [ ] No real WiFi passwords or secrets committed

## Test plan
<!-- How you verified on hardware / bench -->
