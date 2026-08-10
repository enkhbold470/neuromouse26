# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| v2.0.x  | :white_check_mark: |
| < v2.0  | :x:                |

## Reporting a Vulnerability

If you discover a security vulnerability or critical issue within the firmware or web telemetry interface (`WifiDebug.h`), please do not open a public issue.

Instead, please report it privately via [GitHub Security Advisories](https://github.com/enkhbold470/neuromouse26/security/advisories).

We appreciate your effort in responsibly disclosing security findings!

## Credentials

- Do **not** commit real WiFi SSIDs/passwords. `include/WifiDebug.h` ships with
  placeholders (`YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`) and accepts local
  overrides via `#define` / `-D` build flags.
- Rotate any credentials that were previously committed to this repository.
