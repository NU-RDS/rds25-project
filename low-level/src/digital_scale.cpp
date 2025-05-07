#include "HX711.h"
#define SCK 27
#define DT 26

#define BAUD_RATE 9600

#define CALIBRATION_FACTOR -104.26
 
HX711 scale;
// Scale weight reading counter
int count = 1;
 

// Initialising the teensy & load cell
void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long timeout = millis();

  while (!Serial && millis() - timeout < 3000);

  Serial.println("Initializing HX711");

  scale.begin(DT, SCK);
  
  // Calibration values
  scale.set_offset(4294799235);
  scale.set_scale(CALIBRATION_FACTOR);
  
  // Set the scale to 0
  scale.tare();
  Serial.println("Please place weight on scales");
}
 
// Calculating & displaying the weight & average weight readings
void loop() {
  // Reading scale input
  float weight = scale.get_units();
 

  // Checking for valid weight and updating the average
  if (weight > 1) {
    Serial.printf("reading %d: ", count++);
    Serial.println(scale.get_units(), 3);
  }
  delay(500);
}