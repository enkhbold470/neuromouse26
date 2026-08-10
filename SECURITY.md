# Security Policy

## Supported Versions

| Version | Supported |
| ------- | --------- |
| v2.0.x (`main`) | Yes |
| < v2.0 | No |

## Scope

**In scope**

- Firmware under `src/` and `include/` (including dormant `WifiDebug.h` and optional BLE RC)
- Build / CI scripts that affect release integrity
- Optional tools under `tools/` that can command the robot

**Out of scope**

- Third-party PlatformIO packages (NimBLE, U8g2, FastLED, Espressif Arduino)
- Physical chassis theft / tamper
- Competition venue network security

## Lab surfaces (intentional, unauthenticated)

These are **lab / bench tools**, not hardened production services:

- `include/WifiDebug.h` — HTTP debug/tuning API with open CORS; **not** linked into `[env:main]`. Use only on a trusted LAN with placeholder credentials replaced locally.
- `include/BLECarControl.h` — open Nordic UART GATT while the OLED **BLE Car** menu is active; any nearby client can drive motors. Disconnect / leave the menu when not in use.

Treat both as trusted-lab-only. Do not expose them on public networks.

## Reporting a Vulnerability

Do **not** open a public issue.

Report privately via
[GitHub Security Advisories](https://github.com/enkhbold470/neuromouse26/security/advisories/new).

Please include: affected commit/tag, attack scenario, and safe-to-share logs.

## Response

We aim to acknowledge within **7 days** and share a remediation plan when a report is confirmed.

## Credentials

- Do **not** commit real WiFi SSIDs/passwords. `include/WifiDebug.h` ships with
  placeholders (`YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`) and accepts local
  overrides via `#define` / `-D` build flags.
- Rotate any credentials that were previously committed to git history.
- Prefer a gitignored `secrets.h` for local lab credentials.
