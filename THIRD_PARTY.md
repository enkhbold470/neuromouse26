# Third-Party Notices

Project-authored source and original documentation are licensed under the
[MIT License](LICENSE) unless noted below.

## Build-time dependencies (PlatformIO)

These packages are downloaded by PlatformIO at build time (not vendored in this
tree). Redistributors of **firmware binaries** should retain the upstream notices.

| Package | Declared in | License (upstream) | Upstream |
|---|---|---|---|
| NimBLE-Arduino | `platformio.ini` | Apache-2.0 | https://github.com/h2zero/NimBLE-Arduino |
| U8g2 | `platformio.ini` | BSD-2-Clause (library); **fonts have separate licenses** (often SIL OFL) | https://github.com/olikraus/u8g2 |
| FastLED | `platformio.ini` (`main`, `goal-blink`, `ws2812b`) | MIT | https://github.com/FastLED/FastLED |
| Espressif Arduino / ESP-IDF (via `espressif32@6.5.0`) | PlatformIO platform | Apache-2.0 / Espressif terms | https://github.com/espressif/arduino-esp32 |

Fonts used from U8g2 in `include/OLED.h` include `u8g2_font_5x7_tf`,
`u8g2_font_6x10_tf`, `u8g2_font_8x13B_tf`, and `u8g2_font_logisoso42_tn`.
See the [U8g2 font license wiki](https://github.com/olikraus/u8g2/wiki/fntgrp)
for per-font terms.

## Optional tools

| Path | Notes |
|---|---|
| `tools/ble-car-app/` | MIT (same as root); npm lockfile lists transitive MIT / Apache-2.0 / BSD / ISC deps |
| `tools/vision/` | Optional OpenCV + Flask helpers; see `tools/vision/requirements.txt` |

## Media

| Path | Notes |
|---|---|
| `docs/images/*` | Team photos, competition GIFs, and debugger screenshots © 2026 project authors; released under the same MIT terms as this repository |
| Competition award / venue marks | “AAMC”, “IEEE”, and “UCLA” are used factually to describe contest results. **This project is not affiliated with or endorsed by IEEE, UCLA, or AAMC organizers.** |

## External rules

Official All America Micromouse Contest / IEEE micromouse competition rules are
**not redistributed** in this repository. Obtain the current rules from the
contest host (UCLA IEEE Student Branch / AAMC organizers).
