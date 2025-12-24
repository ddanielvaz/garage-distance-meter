# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DistanceMeter is an Arduino Nano-based distance measurement and feedback system using a VL53L0X Time-of-Flight sensor. It displays distance readings on a 4-device 8x8 LED matrix and provides audio feedback via a buzzer when objects are nearby.

## Hardware Setup

**Board:** Arduino Nano

**Pin Configuration:**
- **Distance Sensor (VL53L0X):** I2C communication on A4 (SDA) and A5 (SCL)
- **LED Matrix (4x FC16 modules):** SPI pins - CLK=13, MOSI=11, CS=10
- **Buzzer:** Pin D2 (PWM capable)

**External Libraries:**
- `VL53L0X` (Pololu: https://github.com/pololu/vl53l0x-arduino) - Time-of-Flight distance sensor
- `Wire.h` - I2C communication
- `MD_MAX72xx.h` - LED matrix control

## Architecture

The sketch uses a periodic task pattern with three independent update functions called from `loop()`:

### Core Components

1. **Distance Measurement (`updateDistanceMeasurement`)**
   - Reads from VL53L0X in continuous ranging mode at 50ms intervals
   - Implements rolling average buffer (5 samples) for noise reduction
   - Applies calibration offset (`sensorCalibrationOffset = -53mm`) to raw measurements
   - Returns values in millimeters

2. **LED Matrix Display (`updateMatrixDisplay`)**
   - Updates every 500ms to reduce flicker
   - Converts distance from mm to cm with one decimal place
   - Renders centered text on 32-column display (4 devices × 8 columns)
   - Handles character reversal for correct display orientation
   - Calculates dynamic centering based on text width

3. **Buzzer Feedback (`updateBuzzerFeedback`)**
   - Updates every 50ms using `tone()` function for modulation
   - Distance-based feedback:
     - **< 8cm:** Continuous 2000Hz tone (alarm mode)
     - **8-20cm:** Beeping pattern at 1200Hz (beep rate increases as distance decreases)
     - **> 20cm:** Silent
   - Beep interval mapped from 100-400ms as distance varies

## Configuration Constants

- `DISTANCE_READ_INTERVAL` (50ms) - Sensor read frequency
- `SAMPLE_SIZE` (5) - Rolling average buffer size
- `CONTINUOUS_MODE_TIMEOUT` (1000ms) - VL53L0X timeout
- `sensorCalibrationOffset` (-53mm) - Calibration correction applied to measurements

## Build and Upload

Use Arduino IDE or arduino-cli:

```bash
# Compile
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old DistanceMeter.ino

# Upload (replace /dev/ttyUSB0 with actual port)
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old DistanceMeter.ino

```

## Serial Output

The sketch outputs debug information at 9600 baud:
- Distance readings (in cm with one decimal)
- Character string length for display
- Sensor initialization/error messages

Monitor with: `arduino-cli monitor -p /dev/ttyUSB0`

## Common Modifications

- **Adjust calibration:** Modify `sensorCalibrationOffset` value (in mm)
- **Change buzzer thresholds:** Edit distance comparisons (80mm, 200mm) in `updateBuzzerFeedback()`
- **Adjust beeping frequency/rate:** Modify `beepInterval` mapping and `tone()` frequency values
- **Change display update rate:** Modify `lastUpdateTime` threshold in `updateMatrixDisplay()`
- **Modify sensor read rate:** Change `DISTANCE_READ_INTERVAL` constant

## Timing Notes

- Loop executes continuously with no blocking delays
- All update functions check their own timing internally using `millis()`
- This non-blocking pattern allows responsive sensor reading and display updates simultaneously
