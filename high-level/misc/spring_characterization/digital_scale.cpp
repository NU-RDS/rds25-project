#include <Wire.h>
#include <Adafruit_NAU7802.h>
#include <Arduino.h>

Adafruit_NAU7802 nau;

// Calibration constants
const float ZERO_OFFSET = 102734;
const float NEWTONS_PER_COUNT = 0.000046;

void setup() {
  Serial.begin(115200);
  // Wait for serial port to be available
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);

  if (!nau.begin(&Wire)) {
    Serial.println("NAU7802 not detected");
    while (1);
  }

  // Configure amplifier
  nau.setLDO(NAU7802_3V0);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_320SPS);

  Serial.println("Remove all weight");
  delay(2000);
  nau.calibrate(NAU7802_CALMOD_INTERNAL);
  nau.calibrate(NAU7802_CALMOD_OFFSET);

  // Stabilize ADC
  for (int i = 0; i < 10; i++) {
    while (!nau.available());
    nau.read();
  }

  Serial.println("Start streaming calibrated force values (N)...");
}

void loop() {
  if (nau.available()) {
    int32_t reading = nau.read();
    float force = (reading - ZERO_OFFSET) * NEWTONS_PER_COUNT;

    Serial.print("Force (N): ");
    Serial.println(force, 6);
  }
  delay(100);
}