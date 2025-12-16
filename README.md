# RTES Project

## Overview
This project is designed for Real-Time Embedded Systems (RTES) coursework. It includes embedded code (src/), web interface (web/), and configuration files for PlatformIO and Mbed OS.

## Project Structure

```
RTES Project/
├── include/         # Header files
├── lib/             # Libraries
├── src/             # Source code (main.cpp)
├── test/            # Test files
├── web/             # Web interface (HTML, JS, CSS)
├── mbed_app.json    # Mbed configuration
├── platformio.ini   # PlatformIO config
└── README.md        # Project documentation
```

## ASCII Architecture Diagram

```
+-------------------+
|   Web Interface   |
| (HTML/JS/CSS)     |
+---------+---------+
          |
          | HTTP/WebSocket
          v
+---------+---------+
| Embedded Controller|
|   (main.cpp)      |
+---------+---------+
          |
          | GPIO/UART/I2C/SPI
          v
+---------+---------+
|   Hardware Layer  |
+-------------------+
```

## How to Build & Run
1. Install PlatformIO.
2. Open the project in VS Code.
3. Use PlatformIO to build and upload firmware.
4. Open web/index.html in a browser for the web interface.

## License
MIT License
