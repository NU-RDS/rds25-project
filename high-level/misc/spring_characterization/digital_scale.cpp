#include <Wire.h>
#include <Adafruit_NAU7802.h>
#include <Arduino.h>

Adafruit_NAU7802 nau;

// Calibration constants
int32_t ZERO_OFFSET = 13010;
const float NEWTONS_PER_COUNT = -0.000034;

const unsigned long zero_duration = 5000;

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
  nau.setLDO(NAU7802_3V3);
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

  ZERO_OFFSET = averageReading(zero_duration);
  Serial.print("Zero Offset - ");
  Serial.println(ZERO_OFFSET);

  Serial.println("Start streaming calibrated force values (N)...");

}

void loop() {
  if (nau.available()) {
    int32_t reading = nau.read();
    float force = (reading - ZERO_OFFSET) * NEWTONS_PER_COUNT;

    Serial.print("Force (N): ");
    Serial.println(force);
  }
  delay(100);
}