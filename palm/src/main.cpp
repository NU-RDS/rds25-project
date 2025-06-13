#include <Arduino.h>
#include <SPI.h>  // Ensure SPI library is included for SPI communication

#include <array>
#include <vector>

#include "Encoder.hpp"  // Include the header file for the Encoder class
#include "comms.hpp"

// Encoder chip selects and corresponding IDs
const std::array<int, 8> CS = {2, 3, 4, 5, 6, 14, 15, 18};
const std::array<int, 8> ID = {0, 1, 2, 3, 4, 5, 6, 7};
/**
 * @brief Initializes a vector of Encoder objects, each associated with a unique chip select (CS) and identifier (ID).
 *
 * The encoders vector contains 8 Encoder instances, where each Encoder is constructed
 * using corresponding elements from the CS and ID arrays. This setup is typically used
 * to manage multiple encoder devices connected to a system, allowing for individual
 * control and identification.
 *
 * @note Assumes that the CS and ID arrays are properly defined and contain at least 8 elements each.
 */
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

/**
 * @brief Initializes serial communication, SPI interface, and chip select (CS) pins.
 *
 * This function sets up the serial port at 115200 baud, initializes the SPI interface,
 * and configures each pin in the CS array as an output, setting them HIGH by default.
 * This prepares the microcontroller for SPI communication with multiple devices.
 */
void setup() {
  Serial.begin(115200);
  SPIx.begin(); // Initialize SPI communication
  for (int i = 0; i < CS.size(); i++) {
    pinMode(CS[i], OUTPUT);
    digitalWrite(CS[i], HIGH); // Set CS pins high initially
  }
}

/**
 * @brief Main loop function that reads joint angles from encoders and prints them to the serial monitor.
 *
 * Iterates through all encoders defined in the CS collection, reads the current angle in degrees for each joint,
 * and outputs the encoder ID and corresponding joint angle to the serial monitor. Adds a short delay between readings
 * for each encoder to ensure stable output, and a longer delay at the end of each loop iteration for readability.
 */
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
  
}
