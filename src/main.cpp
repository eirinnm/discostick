/*
  Code for the Discostick project
  Uses Adafruit Bluefruit nRF52, LSM6DS3 IMU, and FastLED library
  This code implements motion controlled LED effects with BLE configurability
  (c) Eirinn Mackay 2025
*/

#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"
extern "C" void* __wrap_calloc(size_t n, size_t size) {
  return calloc(n, size);
}
#include "FastLED.h"

// --- BLE adjustable variables ---
uint8_t fadeOutRate       = 100; // Adjustable via BLE
uint8_t blendFactor       = 100;
uint8_t hueSensitivity10x = 20;   // will be divided by 10 for actual sensitivity
uint8_t accelThreshold    = 2;
uint8_t rotationThreshold = 2;
uint8_t flickerRate       = 25; // Flicker rate in Hz

// uint8_t debugHueSensitivity = 0;

const uint8_t DEFAULT_FADEOUT = 100;
const uint8_t DEFAULT_BLEND = 100;
const uint8_t DEFAULT_HUE = 20;
const uint8_t DEFAULT_ACCEL = 2;
const uint8_t DEFAULT_ROT = 2;
const uint8_t DEFAULT_FLICKER = 25;

// Structure for settings
struct DiscostickSettings {
  uint8_t fadeOutRate;
  uint8_t blendFactor;
  uint8_t hueSensitivity10x;
  uint8_t accelThreshold;
  uint8_t rotationThreshold;
  uint8_t flickerRate;
};

// pin connected to button for toggling flicker effect
#define FLICKER_BUTTON_PIN D8

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
CRGB leds_zero[NUM_LEDS];  // Global LED array set to black, for strobe effect


// Global BLE objects
BLEService controlService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLECharacteristic fadeOutRateChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic blendFactorChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic hueSensitivityChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic accelThresholdChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic rotationThresholdChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic flickerRateChar("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLECharacteristic saveSettingsChar("19B10007-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLECharacteristic restoreDefaultsChar("19B10008-E8F2-537E-4F6C-D104768A1214", BLEWrite);
BLEService batteryService("180F"); // Standard Battery Service UUID
BLECharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

// Function prototypes for settings management
void saveSettings();
void loadSettings();
void restoreDefaults();


void fadeOutRateWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len >= 1) {
        fadeOutRate = data[0];
        Serial.print("Updated fadeOutRate: ");
        Serial.println(fadeOutRate);
    }
}
void blendFactorWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len >= 1) {
        blendFactor = data[0];
        Serial.print("Updated blendFactor: ");
        Serial.println(blendFactor);
    }
}
void hueSensitivityWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len >= 1) {
        hueSensitivity10x = data[0];
        Serial.print("Updated hueSensitivity: ");
        Serial.println(hueSensitivity10x);
    }
}
void accelThresholdWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len >= 1) {
        accelThreshold = data[0];
        Serial.print("Updated accelThreshold: ");
        Serial.println(accelThreshold);
    }
}
void rotationThresholdWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len >= 1) {
        rotationThreshold = data[0];
        Serial.print("Updated rotationThreshold: ");
        Serial.println(rotationThreshold);
    }
}
void flickerRateWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len >= 1) {
    flickerRate = data[0];
    Serial.print("Updated flickerRate: ");
    Serial.println(flickerRate);
  }
}
void saveSettingsWriteCallback(uint16_t, BLECharacteristic*, uint8_t*, uint16_t) {
  saveSettings();
  // flash the LEDs
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(100);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}
void restoreDefaultsWriteCallback(uint16_t, BLECharacteristic*, uint8_t*, uint16_t) {
  restoreDefaults();
  saveSettings(); // Optionally save defaults immediately
}

// Save settings to InternalFileSystem (LittleFS)
void saveSettings() {
  DiscostickSettings s = {
    fadeOutRate,
    blendFactor,
    hueSensitivity10x,
    accelThreshold,
    rotationThreshold,
    flickerRate
  };

  // Remove the file first to ensure truncation
  InternalFS.remove("/discostick.cfg");

  Adafruit_LittleFS_Namespace::File f = InternalFS.open("/discostick.cfg", Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (f) {
    f.write((uint8_t*)&s, sizeof(DiscostickSettings));
    f.close();
    Serial.println("Settings saved to InternalFS.");
  } else {
    Serial.println("Failed to open settings file for writing!");
  }
}

// Load settings from InternalFileSystem (LittleFS)
void loadSettings() {
  Adafruit_LittleFS_Namespace::File f = InternalFS.open("/discostick.cfg", Adafruit_LittleFS_Namespace::FILE_O_READ);
  if (f) {
    Serial.print("discostick.cfg size: ");
    Serial.println(f.size());
  }
  if (f && f.size() == sizeof(DiscostickSettings)) {
    DiscostickSettings s;
    f.read((uint8_t*)&s, sizeof(DiscostickSettings));
    f.close();
    fadeOutRate = s.fadeOutRate;
    blendFactor = s.blendFactor;
    hueSensitivity10x = s.hueSensitivity10x;
    accelThreshold = s.accelThreshold;
    rotationThreshold = s.rotationThreshold;
    flickerRate = s.flickerRate;
    // debugHueSensitivity = s.hueSensitivity10x; // Set debug variable ONCE here
    Serial.println("Settings loaded from InternalFS.");
  } else {
    Serial.println("No valid settings file found, using defaults.");
    restoreDefaults();
    saveSettings(); // Save defaults so file exists for next boot
  }
}

void restoreDefaults() {
  fadeOutRate = DEFAULT_FADEOUT;
  blendFactor = DEFAULT_BLEND;
  hueSensitivity10x = DEFAULT_HUE;
  accelThreshold = DEFAULT_ACCEL;
  rotationThreshold = DEFAULT_ROT;
  flickerRate = DEFAULT_FLICKER;

  // Update BLE characteristics
  fadeOutRateChar.write8(fadeOutRate);
  blendFactorChar.write8(blendFactor);
  hueSensitivityChar.write8(hueSensitivity10x);
  accelThresholdChar.write8(accelThreshold);
  rotationThresholdChar.write8(rotationThreshold);
  flickerRateChar.write8(flickerRate);

  Serial.println("Settings restored to defaults.");
}


void setup() {
  Serial.begin(115200);
   // Wait for serial connection (only if USB is connected)
  // while (!Serial) {
  //   delay(10);
  // }
  Serial.println("Discostick starting up...");
  pinMode(FLICKER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(VBAT_ENABLE, OUTPUT); // set low to enable VBAT reading
  digitalWrite(VBAT_ENABLE, LOW); // Enable VBAT reading
  pinMode(PIN_VBAT, INPUT); // Set VBAT ADC pin as input to read voltage
  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);        // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(12);           // wireing_analog_nRF52.c:39
  // Initialize FastLED.
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.showColor(CRGB::Black);
  FastLED.setBrightness(150);
  FastLED.setCorrection(CRGB(255, 180, 180));
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Initialize zero array
  fill_solid(leds_zero, NUM_LEDS, CRGB::Black); // Initialize zero array
  // Configure the IMU.
  while (myIMU.begin() != 0) {
      delay(1);
  }
  if (!InternalFS.begin()) {
    Serial.println("Failed to mount InternalFS!");
    while (1) { delay(10); }
  }
  Serial.println("InternalFS mounted.");
  // Initialize Bluefruit.
  Bluefruit.begin();
  Bluefruit.autoConnLed(false); // Disable auto connection LED
  Bluefruit.setName("Discostick");
  // Optionally set Tx power
  
  // Begin global service.
  controlService.begin();
  
  
  // Configure and initialize characteristics using native 1-byte values.
  fadeOutRateChar.setFixedLen(1);
  fadeOutRateChar.setWriteCallback(fadeOutRateWriteCallback);
  fadeOutRateChar.setUserDescriptor("Fade-out Rate");
  fadeOutRateChar.begin();
  
  blendFactorChar.setFixedLen(1);
  blendFactorChar.setWriteCallback(blendFactorWriteCallback);
  blendFactorChar.setUserDescriptor("Blend Factor");
  blendFactorChar.begin();
  
  hueSensitivityChar.setFixedLen(1);
  hueSensitivityChar.setWriteCallback(hueSensitivityWriteCallback);
  hueSensitivityChar.setUserDescriptor("Hue Sensitivity");
  hueSensitivityChar.begin();
  
  accelThresholdChar.setFixedLen(1);
  accelThresholdChar.setWriteCallback(accelThresholdWriteCallback);
  accelThresholdChar.setUserDescriptor("Accel Threshold");
  accelThresholdChar.begin();
  
  rotationThresholdChar.setFixedLen(1);
  rotationThresholdChar.setWriteCallback(rotationThresholdWriteCallback);
  rotationThresholdChar.setUserDescriptor("Rotation Threshold");
  rotationThresholdChar.begin();
  
  flickerRateChar.setFixedLen(1);
  flickerRateChar.setWriteCallback(flickerRateWriteCallback);
  flickerRateChar.setUserDescriptor("Flicker Rate");
  flickerRateChar.begin();
  
  saveSettingsChar.setFixedLen(1);
  saveSettingsChar.setUserDescriptor("Save Settings");
  saveSettingsChar.setWriteCallback(saveSettingsWriteCallback);
  saveSettingsChar.begin();
  
  restoreDefaultsChar.setFixedLen(1);
  restoreDefaultsChar.setUserDescriptor("Restore Defaults");
  restoreDefaultsChar.setWriteCallback(restoreDefaultsWriteCallback);
  restoreDefaultsChar.begin();

  batteryService.begin();

  batteryLevelChar.setFixedLen(1);
  batteryLevelChar.setUserDescriptor("Battery Level");
  batteryLevelChar.begin();

  loadSettings(); // Load settings from InternalFS
  fadeOutRateChar.write8(fadeOutRate);
  blendFactorChar.write8(blendFactor);
  hueSensitivityChar.write8(hueSensitivity10x);
  accelThresholdChar.write8(accelThreshold);
  rotationThresholdChar.write8(rotationThreshold);
  flickerRateChar.write8(flickerRate);

  // Begin advertising the service.
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addManufacturerData("Eirinn", 6); 
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(controlService);
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(1600, 3200); // 1–2 seconds
  Bluefruit.Periph.setConnInterval(80, 160);     // 100–200ms (optional)
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

// Use the correct divider factor and reference voltage
#define VBAT_DIVIDER_RATIO (510.0f / (1000.0f + 510.0f)) // ≈ 0.3377
#define VBAT_DIVIDER_CORRECTION (1.0f / VBAT_DIVIDER_RATIO) // ≈ 2.96
#define ADC_REF_VOLTAGE 3.6f // Change to 3.3f if that's correct
#define ADC_MAX 4095.0f      // Change to 4095.0f if 12-bit ADC

uint8_t getBatteryPercent() {
  uint16_t raw = analogRead(PIN_VBAT);
  float vbat_measured = (raw / ADC_MAX) * ADC_REF_VOLTAGE;
  float vbat = vbat_measured * VBAT_DIVIDER_CORRECTION;
  float percent = (vbat - 3.0f) / (4.2f - 3.0f) * 100.0f;
  if (percent > 100.0f) percent = 100.0f;
  if (percent < 0.0f) percent = 0.0f;
  return (uint8_t)(percent + 0.5f);
}

bool checkButtonPress() {
    static bool flickerEnabled = false;
    static unsigned long holdCounter = 0;
    bool currentButtonState = digitalRead(FLICKER_BUTTON_PIN);
    if (currentButtonState == LOW) {
        holdCounter++;
    } else {
        if (holdCounter > 5) { // Button was held for >50ms
            flickerEnabled = !flickerEnabled;
        }
        holdCounter = 0; // Always reset on release
    }
    return flickerEnabled;
}

void loop() {
    unsigned long now = millis();

    // Update sensor data and compute hue.
    uint8_t hue = updateIMU() * hueSensitivity10x / 10; // Scale hue for better visibility.

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
    // Strobing effect based on flickerRate
    bool flickerEnabled = checkButtonPress(); // Check for button press to toggle flicker effect
    static bool strobeOn = true;
    static unsigned long lastStrobe = 0;
    if (flickerEnabled && flickerRate > 0) {
      unsigned long strobeInterval = 500 / flickerRate; // ms for half cycle
      if (now - lastStrobe >= strobeInterval) {
        strobeOn = !strobeOn;
        lastStrobe = now;
      }
    } else {
      strobeOn = true; // If flicker is disabled, always show LEDs
    }
    if (strobeOn){
      FastLED[0].setLeds(leds, NUM_LEDS); // display main LEDs
    } else {
      FastLED[0].setLeds(leds_zero, NUM_LEDS); // Set to zero array for strobe effect
    }
    if (flickerEnabled){
      FastLED.setBrightness(200); // brighter when flicker is enabled so it doesn't dim the effect
    } else {
      FastLED.setBrightness(150); // Use a lower brightness when all LEDs are on
    }
    FastLED.show();
    FastLED.delay(10); // limit frame rate to 100 FPS
    static unsigned long lastBatteryUpdate = 0;
    if (millis() - lastBatteryUpdate > 5000) { // Every 5 seconds
      uint8_t battery = getBatteryPercent();
      batteryLevelChar.write8(battery);      // Update value
      batteryLevelChar.notify8(battery);     // Notify connected clients
      lastBatteryUpdate = millis();
    }
}
