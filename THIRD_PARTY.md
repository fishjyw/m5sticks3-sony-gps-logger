# Third-party components

This project includes or depends on the following open-source components.
Their licenses apply to their respective portions.

| Component | License | Notes |
|---|---|---|
| [furble](https://github.com/gkoh/furble) | MIT | `lib/furble/` — Sony/Canon/Nikon/Fujifilm/Ricoh BLE protocols, patched for Sony A7R4. Copyright (c) 2020 Guo-Rong Koh; see `lib/furble/LICENSE`. |
| [M5Unified](https://github.com/m5stack/M5Unified) | MIT | Board, display, power and button support (PlatformIO dependency). |
| [M5GFX](https://github.com/m5stack/M5GFX) | MIT | Display driver (PlatformIO dependency). |
| [M5PM1](https://github.com/m5stack/M5PM1) | MIT | PMIC driver (PlatformIO dependency). |
| [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) | LGPL-2.1 | NMEA parser, linked as a library (PlatformIO dependency). |
| [LittleFS (esp_littlefs)](https://github.com/joltwallet/esp_littlefs) | BSD-3-Clause | LittleFS VFS (managed component). |
| [littlefs](https://github.com/littlefs-project/littlefs) | BSD-3-Clause | Filesystem implementation. |
| [NimBLE-Arduino / esp-nimble-cpp](https://github.com/h2zero/esp-nimble-cpp) | Apache-2.0 | BLE host stack (managed component). |
| [Apache NimBLE](https://github.com/apache/mynewt-nimble) | Apache-2.0 | BLE host core. |

GNSS initialization commands follow M5Stack's Unit GPS documentation and the
CASIC `$PCAS04` protocol description.

The BLE location packet format (95-byte `sony_geo_t` written to the dd11
characteristic) was derived from public reverse-engineering projects,
including [anoulis/sony_camera_bluetooth_external_gps](https://github.com/anoulis/sony_camera_bluetooth_external_gps)
and [Saschl/alpha-gps](https://github.com/Saschl/alpha-gps).
