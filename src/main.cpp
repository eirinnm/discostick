/*
  Code for the Spectra project (BLE-only build).
  All LSM6DS3 (IMU) and FastLED functionality disabled.
  Using Bluefruit library.
  Now the characteristics store their values as strings,
  and invalid string values (non-numeric) are safely ignored.
*/

#include <Arduino.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"
extern "C" void* __wrap_calloc(size_t n, size_t size) {
  return calloc(n, size);
}
#include "FastLED.h"

// --- BLE adjustable variables ---
uint8_t fadeOutRate       = 220; // Adjustable via BLE
uint8_t blendFactor       = 100;
float   hueSensitivity    = 2.0f;
uint8_t accelThreshold    = 2;
uint8_t rotationThreshold = 2;

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

// Complementary filter parameters
float alpha = 0.98f;  // Complementary filter coefficient
float roll = 0.0f, pitch = 0.0f;
unsigned long lastUpdate = 0;

// Global sensor-derived values for condition checking
float totalAccel = 0.0f;
float totalRotation = 0.0f;

// FastLED setup
#define NUM_LEDS 21
#define DATA_PIN D3 
CRGB leds[NUM_LEDS];  // Global LED array


// Global BLE objects
BLEService controlService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLECharacteristic fadeOutRateChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic blendFactorChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic hueSensitivityChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic accelThresholdChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic rotationThresholdChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// Helper: maximum buffer size for string conversion.
#define BUF_SIZE 16

// Helper function to validate and convert to int.
bool validateInt(const char *str, long *value) {
  char *end;
  long val = strtol(str, &end, 10);
  if (end == str || *end != '\0') {
    return false;
  }
  *value = val;
  return true;
}

// Helper function to validate and convert to float.
bool validateFloat(const char *str, float *value) {
  char *end;
  float val = strtof(str, &end);
  if (end == str || *end != '\0') {
    return false;
  }
  *value = val;
  return true;
}

// Write callback for fadeOutRate characteristic.
void fadeOutRateWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buf[BUF_SIZE];
  if(len < BUF_SIZE) {
    memcpy(buf, data, len);
    buf[len] = '\0';
    long val;
    if(validateInt(buf, &val)) {
      fadeOutRate = (uint8_t)val;
      Serial.print("Updated fadeOutRate: ");
      Serial.println(fadeOutRate);
    } else {
      Serial.println("Invalid fadeOutRate value received. Ignored.");
    }
  }
}

// Write callback for blendFactor characteristic.
void blendFactorWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buf[BUF_SIZE];
  if(len < BUF_SIZE) {
    memcpy(buf, data, len);
    buf[len] = '\0';
    long val;
    if(validateInt(buf, &val)) {
      blendFactor = (uint8_t)val;
      Serial.print("Updated blendFactor: ");
      Serial.println(blendFactor);
    } else {
      Serial.println("Invalid blendFactor value received. Ignored.");
    }
  }
}

// Write callback for hueSensitivity characteristic.
void hueSensitivityWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buf[BUF_SIZE];
  if(len < BUF_SIZE) {
    memcpy(buf, data, len);
    buf[len] = '\0';
    float val;
    if(validateFloat(buf, &val)) {
      hueSensitivity = val;
      Serial.print("Updated hueSensitivity: ");
      Serial.println(hueSensitivity, 2);
    } else {
      Serial.println("Invalid hueSensitivity value received. Ignored.");
    }
  }
}

// Write callback for accelThreshold characteristic.
void accelThresholdWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buf[BUF_SIZE];
  if(len < BUF_SIZE) {
    memcpy(buf, data, len);
    buf[len] = '\0';
    long val;
    if(validateInt(buf, &val)) {
      accelThreshold = (uint8_t)val;
      Serial.print("Updated accelThreshold: ");
      Serial.println(accelThreshold);
    } else {
      Serial.println("Invalid accelThreshold value received. Ignored.");
    }
  }
}

// Write callback for rotationThreshold characteristic.
void rotationThresholdWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buf[BUF_SIZE];
  if(len < BUF_SIZE) {
    memcpy(buf, data, len);
    buf[len] = '\0';
    long val;
    if(validateInt(buf, &val)) {
      rotationThreshold = (uint8_t)val;
      Serial.print("Updated rotationThreshold: ");
      Serial.println(rotationThreshold);
    } else {
      Serial.println("Invalid rotationThreshold value received. Ignored.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Initialize FastLED.
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.showColor(CRGB::Black);
  FastLED.setBrightness(150);
  FastLED.setCorrection(CRGB(255, 180, 180));
  // Configure the IMU.
  while (myIMU.begin() != 0) {
      delay(1);
  }
  // Initialize Bluefruit.
  Bluefruit.begin();
  Bluefruit.setName("Spectra");
  // Optionally set Tx power
  // Bluefruit.setTxPower(4);

  // Get and print the MAC address.
  uint8_t addr[6];
  Bluefruit.getAddr(addr);
  Serial.print("Device MAC: ");
  for (int i = 5; i >= 0; i--) {
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
    if(i > 0) Serial.print(":");
  }
  Serial.println();

  // Begin global service.
  controlService.begin();

  // Configure and initialize characteristics.
  char buf[BUF_SIZE];

  fadeOutRateChar.setFixedLen(0);  // variable length
  fadeOutRateChar.setWriteCallback(fadeOutRateWriteCallback);
  fadeOutRateChar.setUserDescriptor("Fade-out Rate");
  fadeOutRateChar.begin();
  sprintf(buf, "%d", fadeOutRate);
  fadeOutRateChar.write((uint8_t*)buf, strlen(buf));

  blendFactorChar.setFixedLen(0);
  blendFactorChar.setWriteCallback(blendFactorWriteCallback);
  blendFactorChar.setUserDescriptor("Blend Factor");
  blendFactorChar.begin();
  sprintf(buf, "%d", blendFactor);
  blendFactorChar.write((uint8_t*)buf, strlen(buf));

  hueSensitivityChar.setFixedLen(0);
  hueSensitivityChar.setWriteCallback(hueSensitivityWriteCallback);
  hueSensitivityChar.setUserDescriptor("Hue Sensitivity");
  hueSensitivityChar.begin();
  sprintf(buf, "%.2f", hueSensitivity);
  hueSensitivityChar.write((uint8_t*)buf, strlen(buf));

  accelThresholdChar.setFixedLen(0);
  accelThresholdChar.setWriteCallback(accelThresholdWriteCallback);
  accelThresholdChar.setUserDescriptor("Accel Threshold");
  accelThresholdChar.begin();
  sprintf(buf, "%d", accelThreshold);
  accelThresholdChar.write((uint8_t*)buf, strlen(buf));

  rotationThresholdChar.setFixedLen(0);
  rotationThresholdChar.setWriteCallback(rotationThresholdWriteCallback);
  rotationThresholdChar.setUserDescriptor("Rotation Threshold");
  rotationThresholdChar.begin();
  sprintf(buf, "%d", rotationThreshold);
  rotationThresholdChar.write((uint8_t*)buf, strlen(buf));

  // Begin advertising the service.
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(controlService);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.start();

  Serial.println("Bluefruit BLE server started, waiting for connections...");
}


// Helper function to wrap angles to [-180, 180] degrees.
float wrapAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Reads sensor data from the IMU, updates roll and pitch, then returns the calculated hue.
// Also updates totalAccel and totalRotation.
uint8_t updateIMU() {
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;
    
    // Read raw sensor data.
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();
    
    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();
    
    // Convert gyroscope data from degrees/sec to radians/sec.
    const float degToRad = 3.14159265358979f / 180.0f;
    gx *= degToRad;
    gy *= degToRad;
    gz *= degToRad;
    
    // Compute totals for condition checking.
    totalAccel = fabs(ax) + fabs(ay) + fabs(az);
    totalRotation = fabs(gx) + fabs(gy);
    
    // Calculate accelerometer-based Roll and Pitch (in degrees).
    float rollAcc = atan2(ay, az) * 180.0f / PI;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    
    // Complementary filter to update roll and pitch.
    float integratedRoll = roll + gx * dt;
    float diffAngle = wrapAngle180(rollAcc - integratedRoll);
    roll = integratedRoll + (1 - alpha) * diffAngle;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitchAcc;
    
    // Ensure roll stays within -180 to 180.
    roll = wrapAngle180(roll);
    
    // Map roll from [-180, 180] to hue value [0, 255].
    uint8_t hue = (uint8_t)(((roll + 180.0f) / 360.0f) * 256.0f) % 256;
    return hue;
}



void loop() {
    static unsigned long printTimer = 0;
    unsigned long now = millis();

    // Update sensor data and compute hue.
    uint8_t hue = updateIMU() * hueSensitivity; // Scale hue for better visibility.

    // Set the brightness for the first LED based on motion thresholds.
    static uint8_t firstLedBrightness = 0;
    if ((totalAccel > accelThreshold) || (totalRotation > rotationThreshold)) {
        firstLedBrightness = 255;
    } else { 
        firstLedBrightness = scale8(firstLedBrightness, fadeOutRate); // Fade out effect.
    }
    
    // Create trailing LED effect by blending pixels.
    for (int i = NUM_LEDS - 1; i > 0; i--) {
        leds[i] = blend(leds[i], leds[i - 1], blendFactor);
    }
    // Update the first LED with new hue and brightness.
    leds[0] = CHSV(hue, 255, firstLedBrightness);

    FastLED.show();
    FastLED.delay(10); // limit frame rate to 100 FPS

  //   if (millis() - printTimer > 1000) {
  //   Serial.print("fadeOutRate: ");
  //   Serial.print(fadeOutRate);
  //   Serial.print(" | blendFactor: ");
  //   Serial.print(blendFactor);
  //   Serial.print(" | hueSensitivity: ");
  //   Serial.print(hueSensitivity, 2);
  //   Serial.print(" | accelThreshold: ");
  //   Serial.print(accelThreshold);
  //   Serial.print(" | rotationThreshold: ");
  //   Serial.println(rotationThreshold);
  //   printTimer = millis();
  // }
}
