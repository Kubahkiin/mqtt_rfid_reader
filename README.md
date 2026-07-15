# Waveshare ESP32-P4-NANO Ethernet base project

Minimal PioArduino project for the Waveshare ESP32-P4-NANO board.
The application prints basic board information, starts Ethernet, logs link/IP
events, and keeps `loop()` idle for project-specific code.

Keep the file name `platformio.ini`; PioArduino uses the same project format as
PlatformIO.

## Board setup

- Board definition: `boards/waveshare_esp32_p4_nano.json`
- Flash layout: `huge_app_16mb.csv`
- MCU: ESP32-P4NRW32
- PSRAM: 32 MB in package
- Flash: 16 MB NOR Flash

## VS Code setup

Install the recommended workspace extensions:

- `pioarduino.pioarduino-ide`
- `Jason2866.esp-decoder`

Do not use the official `platformio.platformio-ide` extension in this workspace at the same time.
