#include <Arduino.h>
#include "HX711.h"

// Load cell's SCK & DT pins
#define SCK 27
#define DT 26

// Preset known weight (grams)
#define KNOWN_WEIGHT 20.0
#define BAUD_RATE 9600

HX711 scale;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(BAUD_RATE);
  unsigned long timeout = millis();

  while (!Serial && millis() - timeout < 3000);

  Serial.println("Initializing HX711");
  scale.begin(DT, SCK);
}

void loop() {
  if (scale.is_ready()) {

    // LED ON: calibration start
    digitalWrite(LED_BUILTIN, HIGH);

    // Reset scale
    scale.set_scale();
    Serial.println("Tare. Remove any weights from the scale.");
    delay(5000);
    scale.tare();
    Serial.println("Tare done");

    delay(1000);
    Serial.println("Place a known weight on the scale");
    delay(5000);

    float reading = scale.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading, 2);

    float calibration_factor = reading / KNOWN_WEIGHT;
    Serial.print("Calibration factor: ");
    Serial.println(calibration_factor, 2);

    // LED OFF: calibration done
    digitalWrite(LED_BUILTIN, LOW);
  } 
  
  else {

    Serial.println("HX711 not found");
    // Fast blink to indicate error
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);

  }
}
