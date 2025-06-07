/* 
Code for the Spectra project, an IMU-controlled LED device with BLE.
The target board is the Seeeduino XIAO nRF52840 Sense.
*/

#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include "FastLED.h"

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

// LED behavior variables (modifiable via BLE)
uint8_t fadeOutRate   = 220; // Rate at which the first LED fades out (0-255)
uint8_t blendFactor   = 100; // Blending factor for trailing effect
float   hueSensitivity = 2.0f; // Scale factor for hue from roll
uint8_t accelThreshold = 2;   // Threshold for acceleration to trigger brightness
uint8_t rotationThreshold = 2; // Threshold for rotation to trigger brightness

// --- Bluefruit BLE Setup ---
// Define a custom service and characteristics UUIDs
BLEService controlService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLECharacteristic fadeOutRateChar("19B10001-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic blendFactorChar("19B10002-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic hueSensitivityChar("19B10003-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic accelThresholdChar("19B10004-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic rotationThresholdChar("19B10005-E8F2-537E-4F6C-D104768A1214");

// Write callback functions for each characteristic.
void fadeOutRateWriteCallback(BLECharacteristic& chr) {
  fadeOutRate = chr.read8();
  Serial.print("Updated fadeOutRate: ");
  Serial.println(fadeOutRate);
}

void blendFactorWriteCallback(BLECharacteristic& chr) {
  blendFactor = chr.read8();
  Serial.print("Updated blendFactor: ");
  Serial.println(blendFactor);
}

void hueSensitivityWriteCallback(BLECharacteristic& chr) {
  // Read sizeof(float) bytes for hueSensitivity.
  chr.read((uint8_t*)&hueSensitivity, sizeof(hueSensitivity));
  Serial.print("Updated hueSensitivity: ");
  Serial.println(hueSensitivity);
}

void accelThresholdWriteCallback(BLECharacteristic& chr) {
  accelThreshold = chr.read8();
  Serial.print("Updated accelThreshold: ");
  Serial.println(accelThreshold);
}

void rotationThresholdWriteCallback(BLECharacteristic& chr) {
  rotationThreshold = chr.read8();
  Serial.print("Updated rotationThreshold: ");
  Serial.println(rotationThreshold);
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

    // --- Initialize Bluefruit BLE ---
    Bluefruit.begin();
    Bluefruit.setName("Spectra");
    Bluefruit.setTxPower(4);
    
    // Configure our custom service.
    controlService.begin();
    
    // Configure fadeOutRate characteristic.
    fadeOutRateChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    fadeOutRateChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    fadeOutRateChar.setFixedLen(1);
    fadeOutRateChar.setWriteCallback(fadeOutRateWriteCallback);
    fadeOutRateChar.begin();
    fadeOutRateChar.write8(fadeOutRate);

    // Configure blendFactor characteristic.
    blendFactorChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    blendFactorChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blendFactorChar.setFixedLen(1);
    blendFactorChar.setWriteCallback(blendFactorWriteCallback);
    blendFactorChar.begin();
    blendFactorChar.write8(blendFactor);

    // Configure hueSensitivity characteristic (float value).
    hueSensitivityChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    hueSensitivityChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    hueSensitivityChar.setFixedLen(sizeof(hueSensitivity));
    hueSensitivityChar.setWriteCallback(hueSensitivityWriteCallback);
    hueSensitivityChar.begin();
    hueSensitivityChar.write((uint8_t*)&hueSensitivity, sizeof(hueSensitivity));

    // Configure accelThreshold characteristic.
    accelThresholdChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    accelThresholdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    accelThresholdChar.setFixedLen(1);
    accelThresholdChar.setWriteCallback(accelThresholdWriteCallback);
    accelThresholdChar.begin();
    accelThresholdChar.write8(accelThreshold);

    // Configure rotationThreshold characteristic.
    rotationThresholdChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    rotationThresholdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    rotationThresholdChar.setFixedLen(1);
    rotationThresholdChar.setWriteCallback(rotationThresholdWriteCallback);
    rotationThresholdChar.begin();
    rotationThresholdChar.write8(rotationThreshold);
    
    // Begin advertising our service.
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(controlService);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625ms
    Bluefruit.Advertising.start();
    
    Serial.println("Bluefruit BLE device active, waiting for connections...");
    // --- End BLE initialization ---
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
    
    if (now - printTimer >= 20) {
        Serial.print(totalAccel); Serial.print("\t");
        Serial.print(totalRotation); Serial.println();
        printTimer = now;
    }
    
    // Create trailing LED effect by blending pixels.
    for (int i = NUM_LEDS - 1; i > 0; i--) {
        leds[i] = blend(leds[i], leds[i - 1], blendFactor);
    }
    // Update the first LED with new hue and brightness.
    leds[0] = CHSV(hue, 255, firstLedBrightness);

    FastLED.show();
    FastLED.delay(10); // limit frame rate to 100 FPS
}
