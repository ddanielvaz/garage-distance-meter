// Note: Using Pololu VL53L0X library from https://github.com/pololu/vl53l0x-arduino
// Arduino Nano Version
#include <VL53L0X.h>
#include <Wire.h>
#include <MD_MAX72xx.h>

// LED Matrix configuration (Arduino Nano SPI pins)
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN   13  // SPI Clock (SCK)
#define DATA_PIN  11  // SPI MOSI
#define CS_PIN    10  // SPI SS

// Distance sensor configuration
const uint32_t DISTANCE_READ_INTERVAL = 50;   // Read every 50ms (continuous mode is fast)
const uint8_t SAMPLE_SIZE = 5;                 // Rolling average buffer size
const uint16_t CONTINUOUS_MODE_TIMEOUT = 1000; // Timeout for continuous mode

// applied offset in mm to correct measurements
int8_t sensorCalibrationOffset = -53;

// VL53L0X Time-of-Flight sensor (Pololu library)
VL53L0X distanceSensor;
uint16_t sampleBuffer[SAMPLE_SIZE] = {0};     // Rolling average buffer
uint8_t sampleIndex = 0;                       // Current position in buffer
uint16_t lastDistance = 0;
bool distanceSensorReady = false;
bool distanceSensorError = false;

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

void updateMatrixDisplay() {
  static uint32_t lastUpdateTime = 0;

  if (!distanceSensorReady) return;

  // Update display only if 500ms has passed
  if (millis() - lastUpdateTime < 500) return;

  lastUpdateTime = millis();

  // Format distance as decimal string with one decimal place
  float distanceCm = lastDistance / 10.0;
  Serial.println(distanceCm);
  char distanceStr[8];
  dtostrf(distanceCm, 0, 1, distanceStr);  // width=0 (no padding), decimal_places=1
  Serial.println(distanceStr);

  // Calculate text properties
  uint8_t textLen = strlen(distanceStr);
  Serial.print("Textlen: ");
  Serial.println(textLen);
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
}

// ============ Setup and Loop ============
void setup(void) {
  Wire.begin();  // Arduino Nano uses hardware I2C on A4 (SDA) and A5 (SCL)
  Serial.begin(9600);  // Arduino Nano standard baud rate
  delay(500);  // Wait for serial to stabilize
  Serial.println("DistanceMeter starting...");

  // Initialize VL53L0X Time of Flight sensor (distance meter)
  initVL53L0X();
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
  updateMatrixDisplay();
}
