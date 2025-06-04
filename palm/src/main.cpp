#include <Arduino.h>
#include <array>
#include <vector>
#include <SPI.h> // Ensure SPI library is included for SPI communication
#include "encoder.hpp" // Include the header file for the Encoder class

// Encoder chip selects and corresponding IDs
const std::array<int, 8> CS = {2, 3, 4, 5, 6, 14, 15, 18};
const std::array<int, 8> ID = {0, 1, 2, 3, 4, 5, 6, 7};
std::vector<Encoder> encoders = {
    Encoder(CS[0], ID[0]),
    Encoder(CS[1], ID[1]),
    Encoder(CS[2], ID[2]),
    Encoder(CS[3], ID[3]),
    Encoder(CS[4], ID[4]),
    Encoder(CS[5], ID[5]),
    Encoder(CS[6], ID[6]),
    Encoder(CS[7], ID[7])
};
std::array<float, 8> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
  Serial.begin(115200);
  SPIx.begin(); // Initialize SPI communication
  for (int i = 0; i < CS.size(); i++) {
    pinMode(CS[i], OUTPUT);
    digitalWrite(CS[i], HIGH); // Set CS pins high initially
  }
}

void loop() {
  for (size_t i = 0; i < CS.size(); i++) {
    joint_angles[i] = encoders[i].readEncoderDeg(); // Read angle in degrees
    Serial.print("Encoder ID ");
    Serial.print(ID[i]);
    Serial.print(": ");
    Serial.print(joint_angles[i]);
    Serial.print(" degrees\n");
    delay(10);
  }
  Serial.print("\n\n\n");
  delay(1000); // Delay for readability
  
  // SEND JOINT ANGLES THROUGH CAN
  //
  //
}