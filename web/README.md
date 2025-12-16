# PD Detector Web App (BLE)

A tiny static web dashboard that connects to your firmware over BLE and displays live values for:
- Tremor (UUID `0x1910`)
- Dyskinesia (UUID `0x1911`)
- FOG (UUID `0x1912`)

## Requirements

- A Chromium-based browser with Web Bluetooth:
  - Chrome / Edge on desktop
  - Chrome on Android
- A secure context:
  - `http://localhost` **or** `https://...`

## If the board doesn’t show in the browser picker

- Use Chrome or Edge (Safari won’t do Web Bluetooth)
- Make sure you opened the page via `http://localhost:8000` (not a LAN IP)
- On macOS: System Settings → Privacy & Security → Bluetooth → allow your browser
- Ensure the board is advertising (serial log should show `BLE advertising as "PD-Detector"`)
- Disconnect the board from other BLE centrals (e.g. phone apps) and retry

> iPhone Safari typically does not support Web Bluetooth for normal web pages.

## Run locally

From the project root:

```bash
cd web
python3 -m http.server 8000
```

Then open:

- http://localhost:8000

Click **Connect**, select `PD-Detector`, and you should see live updates.

## Data format

These payloads match the packed structs in `src/main.cpp`.

- Tremor/Dyskinesia (6 bytes):
  - `detected` (uint8)
  - `intensity` (uint8)
  - `freqX100` (uint16 little-endian) => `freq_hz = freqX100 / 100`
  - `power` (uint16 little-endian)

- FOG (4 bytes):
  - `detected` (uint8)
  - `intensity` (uint8)
  - `reserved` (uint16)
