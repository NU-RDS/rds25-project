#include "encoder.hpp"
#include <SPI.h>
#include <Arduino.h>

// Define CS pins for the two encoders
#define ENCODER1_CS 10
#define ENCODER2_CS 37

// Create encoder instances
Encoder encoder1(ENCODER1_CS, 1);  // ID 1
Encoder encoder2(ENCODER2_CS, 2);  // ID 2

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Initialize SPI bus
  SPI.begin();
  
  // Set CS pins as outputs and set them HIGH (inactive)
  pinMode(ENCODER1_CS, OUTPUT);
  pinMode(ENCODER2_CS, OUTPUT);
  digitalWrite(ENCODER1_CS, HIGH);
  digitalWrite(ENCODER2_CS, HIGH);
  
  Serial.println("Encoder Test - Reading 2 encoders on same SPI bus");
}

void loop() {
  // Read values from both encoders
  float angle1 = encoder1.readEncoderDeg();
  float angle2 = encoder2.readEncoderDeg();
  
  // Display the readings
  Serial.print("Encoder 1 (Pin ");
  Serial.print(ENCODER1_CS);
  Serial.print("): ");
  Serial.print(angle1);
  Serial.println(" degrees");
  
  Serial.print("Encoder 2 (Pin ");
  Serial.print(ENCODER2_CS);
  Serial.print("): ");
  Serial.print(angle2);
  Serial.println(" degrees");
  

  Serial.println();
  delay(100);  // Update once per second
}