#include <Wire.h>
#include <Adafruit_NAU7802.h>
#include <Arduino.h>

Adafruit_NAU7802 nau;

// Calibration variables
int32_t ZERO_OFFSET = 0;
float NEWTONS_PER_COUNT = 0.0;

// Constants
// Known mass in grams
const float KNOWN_MASS = 20.0;

// acc due to gravity
const float g = 9.81;

// Calibration duration
const unsigned long calibration_duration = 5000;

int32_t averageReading(unsigned long duration_ms) {
  unsigned long start = millis();
  int32_t sum = 0;
  int count = 0;
  while (millis() - start < duration_ms) {
    if (nau.available()) {
      sum += nau.read();
      count++;
    }
  }
  return (count > 0) ? sum / count : 0;
}

void calibrateLoadCell() {
  
  Serial.println("Step 1: Remove all weight. Calibrating zero offset...");
  delay(2000);
  ZERO_OFFSET = averageReading(calibration_duration);
  Serial.print("Zero offset: ");
  Serial.println(ZERO_OFFSET);
  
  Serial.println("Step 2: Place known weight. Calibrating scale factor...");
  delay(5000);
  int32_t load_avg = averageReading(calibration_duration);
  Serial.print("Loaded average: ");
  Serial.println(load_avg);
  
  int32_t delta = load_avg - ZERO_OFFSET;
  float force = KNOWN_MASS * g * 1e-3;
  NEWTONS_PER_COUNT = force / delta;
  Serial.print("Newtons per count: ");
  Serial.println(NEWTONS_PER_COUNT, 6);
  
  Serial.println("Calibration complete.\n");
}

void setup() {
  Serial.begin(115200);
  // Wait for serial port to be available
  while (!Serial);
  
  Wire.begin();
  Wire.setClock(400000);
  
  if (!nau.begin(&Wire)) {
    Serial.println("NAU7802 not found!");
    while (1);
  }
  
  // Configure amplifier
  nau.setLDO(NAU7802_3V0);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_320SPS);
  nau.calibrate(NAU7802_CALMOD_INTERNAL);
  nau.calibrate(NAU7802_CALMOD_OFFSET);
  
  // Stabilize ADC
  for (int i = 0; i < 10; i++) {
    while (!nau.available());
    nau.read();
  }

}

void loop() {
  calibrateLoadCell();
  delay(1000);
}