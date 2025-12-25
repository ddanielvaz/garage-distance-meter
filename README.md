# DistanceMeter

An Arduino Nano-based distance measurement system using a VL53L0X Time-of-Flight sensor. Displays distance readings on a 4-device 8x8 LED matrix with audio feedback via buzzer.

## LED Matrix Display Rules

The LED matrix displays distance readings and activates/deactivates based on the following rules:

### Activation
- **Turns ON** when distance is less than 50cm
- Shows distance in centimeters with one decimal place (e.g., "12.3")
- Updates every 500ms to reduce flicker

### Deactivation
- **Turns OFF** when distance is greater than 50cm
- Saves power by not updating the display

### Idle Shutdown
- **Clears display** when no significant movement is detected for 15 seconds
- Triggered when distance change is less than 2cm over the 15-second period
- Display remains cleared until movement is detected again (≥2cm change)

## Buzzer Feedback Rules

The buzzer provides audio feedback based on distance and system state:

### Active Modes (when movement is detected)

#### Continuous Alarm
- **Distance < 8cm**: Continuous 2000Hz tone
- Warns of very close proximity

#### Beeping Pattern
- **Distance 8cm - 35cm**: Intermittent 1200Hz beeps
- Beep rate increases as object gets closer:
  - 8cm: ~100ms interval (fastest)
  - 35cm: ~500ms interval (slowest)
- Provides progressive warning as distance decreases

#### Silent
- **Distance > 35cm**: No sound

### Idle Shutdown
- **Silent** when no movement detected for 15 seconds
- Buzzer turns off regardless of distance
- Resumes normal operation when movement is detected (≥2cm change)

## Movement Detection

Movement is detected by tracking distance changes:
- **Significant movement**: ≥2cm (≥20mm) change from last recorded position
- **Idle period**: No significant movement for 15 seconds
- **Wake-up trigger**: Any ≥2cm change while idle immediately restarts the system

## System State Transitions

```
ACTIVE STATE
    ↓
(no movement for 15s)
    ↓
IDLE STATE (buzzer off, display cleared)
    ↓
(movement detected ≥2cm)
    ↓
ACTIVE STATE
```

## Configuration

To adjust idle shutdown behavior, modify these constants in the sketch:

- `IDLE_SHUTDOWN_THRESHOLD` (default: 20mm = 2cm): Minimum distance change to trigger activity
- `IDLE_SHUTDOWN_TIMEOUT` (default: 15000ms): Period before entering idle mode

To adjust display/buzzer thresholds:

- `MATRIX_ACTIVATION_THRESHOLD` (default: 500mm = 50cm): Distance to activate LED matrix
- `BUZZER_CONTINUOUS_THRESHOLD` (default: 80mm = 8cm): Distance for continuous alarm
- `BUZZER_MAX_RANGE` (default: 350mm = 35cm): Maximum distance for beeping pattern

## Serial Debug Output

The system outputs debug messages at 9600 baud:
- Sensor initialization status
- "System idle - shutting down buzzer and display" when idle mode is entered
- "System resumed - movement detected" when movement wakes the system
- Distance readings and display information

Monitor with: `arduino-cli monitor -p /dev/ttyUSB0`
