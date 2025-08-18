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
// #include <Adafruit_NeoPixel.h>
// overriding the LED limit: the following line must be added to clockless_arm_nrf52.h
#define FASTLED_NRF52_MAXIMUM_PIXELS_PER_STRING 160
#include "FastLED.h"

// --- Motion variables ---
uint8_t fadeOutRate       = 150; 
uint8_t speedFactor       = 15;
uint8_t hueSensitivity10x = 10;   // will be divided by 10 for actual sensitivity
uint8_t accelThreshold    = 2;
uint8_t rotationThreshold = 2;
uint8_t rotationThresholdExitScreensaver = rotationThreshold * 4;
uint8_t flickerRate       = 25; // Flicker rate in Hz
uint8_t globalBrightness = 120; 

unsigned long screensaverTimeout_ms = 30000; // 30 seconds


// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

// Complementary filter parameters
float alpha = 0.80f;  // Complementary filter coefficient
float roll = 0.0f, pitch = 0.0f;
unsigned long lastUpdate = 0;

// Global sensor-derived values for condition checking
float totalAccel = 0.0f;
float totalRotation = 0.0f;

// FastLED setup
#define NUM_LEDS 160
// Define data pins for each strip
#define DATA_PIN_1 2
#define DATA_PIN_2 3

CRGB leds[NUM_LEDS];  // Virtual LED array
CRGB leds1[NUM_LEDS];  // LED array for strip 1
CRGB leds2[NUM_LEDS];  // LED array for strip 2


// Global BLE objects
BLEService controlService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEService batteryService("180F"); // Standard Battery Service UUID
BLECharacteristic batteryLevelChar("2A19", BLERead | BLENotify, 1, &batteryService);

void drawFractionalBar(int pos16, int width, uint8_t hue, uint8_t sat);



void setup() {
  Serial.begin(115200);
  // Wait for serial connection (only if USB is connected)
  // while (!Serial) {
  //   delay(10);
  // }
  Serial.println("Discostick starting up...");
  // pinMode(FLICKER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VBAT_ENABLE, OUTPUT); // set low to enable VBAT reading
  digitalWrite(VBAT_ENABLE, LOW); // Enable VBAT reading
  pinMode(PIN_VBAT, INPUT); // Set VBAT ADC pin as input to read voltage
  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);        // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(12);           // wireing_analog_nRF52.c:39
  FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leds1, NUM_LEDS);
  FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(leds2, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 300);
  // FastLED.setBrightness(globalBrightness);
  FastLED.setCorrection(TypicalLEDStrip);
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Initialize zero array
  fill_solid(leds1, NUM_LEDS, CRGB::Black); // Initialize zero array
  fill_solid(leds2, NUM_LEDS, CRGB::Black); // Initialize zero array
  FastLED.show(); // Ensure all LEDs are off initially

  // Configure the IMU.
  while (myIMU.begin() != 0) {
      delay(1);
  }
  setupBLE(); // Initialize BLE services and characteristics
}


void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setName("Discostick");
  Bluefruit.setTxPower(4);
  Bluefruit.setConnectionInterval(100);

  // Add services
  Bluefruit.addService(controlService);
  Bluefruit.addService(batteryService);

  // Set up battery level characteristic
  batteryLevelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryLevelChar.setFixedLen(1); // Battery level is 1 byte
  batteryLevelChar.write8(getBatteryPercent()); // Set initial value

  // Advertise battery service UUID for Android compatibility
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.start();

  Serial.println("BLE setup complete");
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

uint8_t colour_pulse(uint8_t hue){
  // Main animation.
  // Set the brightness for the first LED based on motion thresholds.
  static uint8_t firstLedBrightness = 0;
  if ((totalAccel > accelThreshold) || (totalRotation > rotationThreshold)) {
      firstLedBrightness = 255;
  } else { 
      firstLedBrightness = scale8(firstLedBrightness, fadeOutRate); // Fade out effect.
  }

  // Shift the LED arrays by n=speedFactor positions
  for (int i = NUM_LEDS - 1; i >= speedFactor; i--) {
    leds[i] = leds[i - speedFactor];
  }
  // Fill the first n=speedFactor LEDs with the new color/brightness
  for (int i = 0; i < speedFactor; i++) {
    leds[i] = CHSV(hue, 255, firstLedBrightness);
  }
  // Now copy the virtual LED array to the actual LED arrays
  // Alternate which strip is displayed, each frame
  static bool showPrimaryStrip = true;
  if (showPrimaryStrip) {
    // Show leds1, turn off leds2 (just for display)
  memcpy(leds1, leds, sizeof(leds1));
    fill_solid(leds2, NUM_LEDS, CRGB::Black);
  } else {
    // Show leds2, turn off leds1 (just for display)
  memcpy(leds2, leds, sizeof(leds2));
    fill_solid(leds1, NUM_LEDS, CRGB::Black);
  }
  showPrimaryStrip = !showPrimaryStrip; // Toggle for next frame
  return firstLedBrightness; // Return brightness as indicator of activity
}

// void screensaver() {
//   // Simple color wipe effect for screensaver
//   static uint8_t hue = 0;
//   hue += 5; // Increment hue
//   fill_solid(leds1, NUM_LEDS, CHSV(hue, 255, 255));
//   fill_solid(leds2, NUM_LEDS, CHSV(hue+128, 255, 255));
// }

struct raindrop {
    int pos16 = 0;
    float speed = 0;
    bool active = false;
};
const byte NUM_DROPS = 4;
raindrop drops[NUM_DROPS];
const float RAINDROP_GRAVITY = -0.6; //higher makes it fall faster

const byte NEWDROPS = 5;

void raindrops() {
  // create some falling sprites. They should fall based on gravity.
  // If the IMU reports pitch > 0, drops should start at the end of the strip and move down
  // If the IMU reports pitch near 0, drops should move more slowly.
  static byte masterhue=0;
  static unsigned long lastHueChange = 0;
  fadeToBlackBy(leds, NUM_LEDS, 32);
  if(millis()-lastHueChange > 10){
    masterhue++;
    lastHueChange = millis();
  }
  // get gravity as a float
  float gravity = sin(pitch * PI / 180.0f); // -1 (down) to +1 (up)
  for (int i = 0; i < NUM_DROPS; i++) {
      if (drops[i].active)
      { // this is an active drop, accelerate it
        // note: gravity is a float ranging from -1.0 (pointing down) to +1.0 (straight up)
          drops[i].speed += RAINDROP_GRAVITY * gravity;
          // move it
          drops[i].pos16 += drops[i].speed;
          // if it hits the bottom it should reverse and slow down
          if (drops[i].pos16 <= 0) {
              drops[i].pos16 *= -1;
              // make it bounce by a reduced amount
              drops[i].speed *= -(0.4 + random8(4)/10.0);
          }
          if (drops[i].pos16 > (NUM_LEDS-1)*16) { 
              // if it hits the top, make it bounce
              drops[i].pos16 = (NUM_LEDS-1)*16*2 - drops[i].pos16-1;
              // make it bounce by a reduced amount
              drops[i].speed *= -(0.4 + random8(4)/10.0);
          }
          // if raindrop has bounced enough times that it becomes slow, kill it
          if ((abs(drops[i].speed) < 0.5) && ((drops[i].pos16 == 0) || (drops[i].pos16 >= (NUM_LEDS-1)*16))){
              drops[i].active = false; 
          }
      } else {
          // inactive drop. Should we activate it?
          if(random8() < NEWDROPS){
              if (gravity>0){
                  drops[i].pos16 = (NUM_LEDS-1) * 16; //put it at the top of the staff
              }else{
                  drops[i].pos16 = 1; //put it at the bottom of the staff
              }
              drops[i].active = true;
              drops[i].speed = 0;
          }
      }
      if (drops[i].active) {
        // actually render the droplet
        uint8_t dropHue = masterhue + i*4; // offset hue for each drop
        drawFractionalBar(drops[i].pos16, 3, dropHue, 255);
      }
  }
  // copy virtual array onto physical array (just one to save power)
  memcpy(leds1, leds, sizeof(leds1));
  // memcpy(leds2, leds, sizeof(leds2));
  fill_solid(leds2, NUM_LEDS, CRGB::Black);
}


void loop() {
  unsigned long now = millis();
  static bool screensaverMode = false;
  static unsigned long lastActivityTime = 0;

  // Update sensor data and compute hue.
  uint8_t hue = updateIMU() * hueSensitivity10x / 10; // Scale hue
  if(!screensaverMode) {
    uint8_t activity = colour_pulse(hue); // main animation effect
    // Check for inactivity and switch to screensaver if needed
    if (activity == 0) {
      if (now - lastActivityTime > screensaverTimeout_ms) {
        screensaverMode = true;
      }
    } else {
      lastActivityTime = now;
    }
  }else{
    raindrops();
    // exit screensaver by fast rotation sustained for 2 seconds
    static unsigned long screensaverExitTime = 0;
    if (totalRotation > rotationThresholdExitScreensaver) {
      if (screensaverExitTime == 0) {
        screensaverExitTime = millis();
      }
      if (millis() - screensaverExitTime > 1000) {
        screensaverMode = false;
        screensaverExitTime = 0;
      }
    } else {
      screensaverExitTime = 0;
    }
  }

  FastLED.show(); // Show both strips
  FastLED.delay(10); // limit frame rate to 100 FPS

  static unsigned long lastBatteryUpdate = 0;
  if (millis() - lastBatteryUpdate > 5000) { // Every 5 seconds
    uint8_t battery = getBatteryPercent();
    batteryLevelChar.write8(battery);      // Update value
    batteryLevelChar.notify8(battery);     // Notify connected clients
    lastBatteryUpdate = millis();
  }
}

void drawFractionalBar(int pos16, int width, uint8_t hue, uint8_t sat) {
    int i = (pos16 / 16);  // convert from pos to raw pixel number
    uint8_t frac = pos16 & 0x0F;             // extract the 'factional' part of the position

    // brightness of the first pixel in the bar is 1.0 - (fractional part of position)
    // e.g., if the light bar starts drawing at pixel "57.9", then
    // pixel #57 should only be lit at 10% brightness, because only 1/10th of it
    // is "in" the light bar:
    //
    //                       57.9 . . . . . . . . . . . . . . . . . 61.9
    //                        v                                      v
    //  ---+---56----+---57----+---58----+---59----+---60----+---61----+---62---->
    //     |         |        X|XXXXXXXXX|XXXXXXXXX|XXXXXXXXX|XXXXXXXX |
    //  ---+---------+---------+---------+---------+---------+---------+--------->
    //                   10%       100%      100%      100%      90%
    //
    // the fraction we get is in 16ths and needs to be converted to 256ths,
    // so we multiply by 16.  We subtract from 255 because we want a high
    // fraction (e.g. 0.9) to turn into a low brightness (e.g. 0.1)
    uint8_t firstpixelbrightness = 255 - (frac * 16);

    // if the bar is of integer length, the last pixel's brightness is the
    // reverse of the first pixel's; see illustration above.
    uint8_t lastpixelbrightness = 255 - firstpixelbrightness;

    // For a bar of width "N", the code has to consider "N+1" pixel positions,
    // which is why the "<= width" below instead of "< width".

    uint8_t bright;
    for (int n = 0; n <= width; n++) {
        if (n == 0) {
            // first pixel in the bar
            bright = firstpixelbrightness;
        } else if (n == width) {
            // last pixel in the bar
            bright = lastpixelbrightness;
        } else {
            // middle pixels
            bright = 255;
        }

        leds[i] += CHSV(hue, sat, bright);
        i++;
        if (i >= NUM_LEDS) break;
    }
}