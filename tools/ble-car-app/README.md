# mm26 BLE Car App

Optional Web Bluetooth remote for the Micromouse26 OLED **BLE Car** menu mode
(Nordic UART Service).

## Requirements

- Node.js ≥ 18.13
- Chrome / Edge (Web Bluetooth). Safari is not supported.
- Secure context: `https://` or `http://localhost`

## Setup

```bash
cd tools/ble-car-app
npm ci
npm run dev      # Vite dev server (--host for phone testing on LAN)
```

Production static build:

```bash
npm run build
npm start        # sirv on 0.0.0.0:8000
```

## Security note

While the robot is in BLE Car mode, any nearby client that connects can command
motors. Leave the menu (or power down) when not actively driving. See
[`SECURITY.md`](../../SECURITY.md).

## License

MIT — same as the repository root.
