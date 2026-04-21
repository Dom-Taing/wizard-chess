# Wizard Chess

A CoreXY motion platform that moves to chess board squares by name, built on the Seeed XIAO ESP32-S3.

## Hardware

- **Board:** Seeed XIAO ESP32-S3
- **Motion:** CoreXY belt-drive mechanism (41 cm × 41 cm travel, 16400 steps per axis)
- **Motors:** Two stepper motors
  - Motor A: pins D4 (step), D5 (dir), D6 (enable)
  - Motor B: pins D9 (step), D8 (dir), D7 (enable)

## Usage

Flash the firmware via PlatformIO, then open a serial monitor at **115200 baud**.

Type a chess square and press Enter:

```
A1
```

The platform moves to that square and prints the position:

```
A1 -> (3.80 cm, 5.50 cm)
arrived at (3.80 cm, 5.50 cm)
```

Valid input: columns **A–H**, rows **1–8** (case-insensitive).

## Square Coordinates

The origin square A1 is at **(3.8 cm, 5.5 cm)** from the home position. Each step is **5.0 cm** in both axes.

| | A | B | C | ... |
|---|---|---|---|---|
| **8** | (3.8, 40.5) | (8.8, 40.5) | ... | |
| **1** | (3.8, 5.5) | (8.8, 5.5) | ... | |

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run --target upload
```

Dependencies are managed automatically via `platformio.ini`:
- Adafruit VEML7700 Library
