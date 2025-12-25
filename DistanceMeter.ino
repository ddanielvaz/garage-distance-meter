// Note: Using Pololu VL53L0X library from https://github.com/pololu/vl53l0x-arduino
// Arduino Nano Version
#include <VL53L0X.h>
#include <Wire.h>
#include <MD_MAX72xx.h>

// Buzzer configuration
#define BUZZER_PIN 2  // D2 for buzzer
#define BUZZER_CONTINUOUS_THRESHOLD 80  // Distance in mm for continuous sound (8cm)
#define BUZZER_MAX_RANGE 350  // Distance in mm for beeping pattern upper limit (35cm)

// LED Matrix configuration (Arduino Nano SPI pins)
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN   13  // SPI Clock (SCK)
#define DATA_PIN  11  // SPI MOSI
#define CS_PIN    10  // SPI SS
#define MATRIX_ACTIVATION_THRESHOLD 500  // Distance in mm to activate LED matrix (50cm)

// Distance sensor configuration
const uint32_t DISTANCE_READ_INTERVAL = 50;   // Read every 50ms (continuous mode is fast)
const uint8_t SAMPLE_SIZE = 5;                 // Rolling average buffer size
const uint16_t CONTINUOUS_MODE_TIMEOUT = 1000; // Timeout for continuous mode

// Idle shutdown configuration
const uint16_t IDLE_SHUTDOWN_THRESHOLD = 20;   // Distance change in mm to trigger activity (1cm)
const uint32_t IDLE_SHUTDOWN_TIMEOUT = 15000;  // Time in ms before shutdown (15 seconds)

// applied offset in mm to correct measurements
int8_t sensorCalibrationOffset = -53;

// VL53L0X Time-of-Flight sensor (Pololu library)
VL53L0X distanceSensor;
uint16_t sampleBuffer[SAMPLE_SIZE] = {0};     // Rolling average buffer
uint8_t sampleIndex = 0;                       // Current position in buffer
uint16_t lastDistance = 0;
bool distanceSensorReady = false;
bool distanceSensorError = false;

// Idle shutdown tracking
uint16_t lastSignificantDistance = 0;          // Last distance with significant change
uint32_t lastSignificantChangeTime = 0;        // Timestamp of last significant distance change
bool isSystemIdle = false;                     // System idle state

// LED Matrix display
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
uint16_t lastDisplayedDistance = 0xFFFF;  // Track last displayed value to avoid unnecessary updates


// ============ VL53L0X Functions ============
void initVL53L0X() {
  if (!distanceSensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    distanceSensorReady = false;
    distanceSensorError = true;
    return;
  }

  // Set timeout for continuous ranging mode
  distanceSensor.setTimeout(CONTINUOUS_MODE_TIMEOUT);

  // Start continuous ranging mode for better accuracy and stability
  distanceSensor.startContinuous();

  distanceSensorReady = true;
  distanceSensorError = false;
  Serial.println("VL53L0X sensor initialized in continuous ranging mode");
}

void updateDistanceMeasurement() {
  static uint32_t lastReadTime = 0;

  if (!distanceSensorReady) return;

  if (millis() - lastReadTime >= DISTANCE_READ_INTERVAL) {
    lastReadTime = millis();

    // Read from continuous ranging mode
    uint16_t distance = distanceSensor.readRangeContinuousMillimeters();

    if (distanceSensor.timeoutOccurred()) {
      distanceSensorError = true;
      Serial.println("VL53L0X timeout!");
    } else {
      distanceSensorError = false;

      // Add to rolling average buffer
      sampleBuffer[sampleIndex] = distance;
      sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;

      // Calculate rolling average from all samples
      uint32_t sum = 0;
      for (uint8_t i = 0; i < SAMPLE_SIZE; i++) {
        sum += sampleBuffer[i];
      }
      lastDistance = sum / SAMPLE_SIZE;
    }
    // applying setup offset
    lastDistance += sensorCalibrationOffset;
  }
}

void updateIdleShutdown() {
  // Check if distance has changed significantly
  uint16_t distanceChange = abs((int)lastDistance - (int)lastSignificantDistance);

  if (distanceChange >= IDLE_SHUTDOWN_THRESHOLD) {
    // Significant movement detected, reset idle state
    lastSignificantDistance = lastDistance;
    lastSignificantChangeTime = millis();

    // If system was idle, wake it up
    if (isSystemIdle) {
      isSystemIdle = false;
      Serial.println("System resumed - movement detected");
    }
  } else {
    // No significant movement, check if idle timeout has been reached
    if (millis() - lastSignificantChangeTime >= IDLE_SHUTDOWN_TIMEOUT) {
      if (!isSystemIdle) {
        isSystemIdle = true;
        Serial.println("System idle - shutting down buzzer and display");
      }
    }
  }
}

void updateMatrixDisplay() {
  static uint32_t lastUpdateTime = 0;
  static bool matrixActive = false;

  if (!distanceSensorReady) return;

  // Clear display when system is idle
  if (isSystemIdle) {
    if (matrixActive) {
      mx.clear();
      matrixActive = false;
    }
    return;
  }

  uint16_t distanceInMm = lastDistance;

  // Check if matrix should be active based on distance threshold
  if (distanceInMm < MATRIX_ACTIVATION_THRESHOLD) {
    // Distance is close, matrix should be on
    if (!matrixActive) {
      mx.clear();  // Turn on and clear the display
      matrixActive = true;
    }

    // Update display only if 500ms has passed
    if (millis() - lastUpdateTime < 500) return;
    lastUpdateTime = millis();

    // Format distance as decimal string with one decimal place
    float distanceCm = distanceInMm / 10.0;
    char distanceStr[8];
    dtostrf(distanceCm, 0, 1, distanceStr);  // width=0 (no padding), decimal_places=1

    // Calculate text properties
    uint8_t textLen = strlen(distanceStr);
    const uint8_t CHAR_SPACING = 1;

    // Reverse the string for correct display order
    for (uint8_t i = 0; i < textLen / 2; i++) {
      char temp = distanceStr[i];
      distanceStr[i] = distanceStr[textLen - 1 - i];
      distanceStr[textLen - 1 - i] = temp;
    }

    // Calculate actual text width
    uint8_t cBuf[8];
    uint16_t actualWidth = 0;
    for (uint8_t i = 0; i < textLen; i++) {
      uint8_t charWidth = mx.getChar(distanceStr[i], sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
      actualWidth += charWidth;
      if (i < textLen - 1) actualWidth += CHAR_SPACING;
    }
    int16_t startCol = (32 - actualWidth) / 2;

    // Clear the display
    mx.clear();

    // Draw each character at calculated position
    uint8_t col = startCol;

    for (uint8_t i = 0; i < textLen; i++) {
      uint8_t charWidth = mx.getChar(distanceStr[i], sizeof(cBuf) / sizeof(cBuf[0]), cBuf);

      // Write character columns in reverse order
      for (int8_t j = charWidth - 1; j >= 0; j--) {
        if (col >= 0 && col < 32) {
          mx.setColumn(col, cBuf[j]);
        }
        col++;
      }
      // Add spacing between characters
      col += CHAR_SPACING;
    }
  } else {
    // Distance is far, keep matrix off to save power (skip display updates)
    if (matrixActive) {
      mx.clear();  // Clear display
      matrixActive = false;
    }
  }
}

void updateBuzzerFeedback() {
  static uint32_t lastBuzzerUpdate = 0;
  static uint32_t beepTimer = 0;
  static bool isBuzzing = false;
  const uint8_t minFreq = 25;

  if (!distanceSensorReady) {
    noTone(BUZZER_PIN);
    return;
  }

  // Shut down buzzer when system is idle
  if (isSystemIdle) {
    noTone(BUZZER_PIN);
    isBuzzing = false;
    return;
  }

  // Update every 25ms
  if (millis() - lastBuzzerUpdate < minFreq) return;
  lastBuzzerUpdate = millis();

  uint16_t distanceInMm = lastDistance;

  if (distanceInMm < BUZZER_CONTINUOUS_THRESHOLD) {
    // Continuous sound for very close
    tone(BUZZER_PIN, 2000);
  }
  else if (distanceInMm < BUZZER_MAX_RANGE) {
    // Beeping pattern - speed increases as distance decreases
    // Closer = faster beeps
    uint16_t beepInterval = map(distanceInMm, BUZZER_CONTINUOUS_THRESHOLD, BUZZER_MAX_RANGE, minFreq, 500);

    if (millis() - beepTimer >= beepInterval) {
      isBuzzing = !isBuzzing;
      isBuzzing ? tone(BUZZER_PIN, 1200) : noTone(BUZZER_PIN);
      beepTimer = millis();
    }
  }
  else {
    // Silent when far
    noTone(BUZZER_PIN);
    isBuzzing = false;
  }
}

// ============ Setup and Loop ============
void setup(void) {
  Wire.begin();  // Arduino Nano uses hardware I2C on A4 (SDA) and A5 (SCL)
  Serial.begin(9600);  // Arduino Nano standard baud rate
  delay(500);  // Wait for serial to stabilize
  Serial.println("DistanceMeter starting...");

  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);  // Ensure buzzer is off

  // Initialize VL53L0X Time of Flight sensor (distance meter)
  initVL53L0X();
  // Initialize idle shutdown tracking
  lastSignificantChangeTime = millis();
  // Initialize LED Matrix
  if (!mx.begin()) {
    Serial.println("LED Matrix failed to start");
  } else {
    mx.clear();
    Serial.println("LED Matrix initialized successfully");
  }
}

void loop(void) {
  updateDistanceMeasurement();
  updateIdleShutdown();
  updateMatrixDisplay();
  updateBuzzerFeedback();
}
