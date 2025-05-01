#include <Arduino.h>
#include <SPI.h>
#include "force_control.hpp"

// Define constants
#define NUM_TENDONS 2

// Structure to hold tendon configuration
struct TendonConfig {
  int csPin;
  int encoderId;
  ForceType forceType;
  ForceControl* controller;
};

// Array of tendon configurations
TendonConfig tendons[NUM_TENDONS];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("Force Control System Initializing...");
  
  // Initialize SPI
  SPI.begin();
  
  // Initialize tendons with default values
  // You can modify these parameters as needed
  float ff = 1.0f;
  float kp = 0.8f;
  float ki = 0.2f;
  float kd = 0.05f;
  float ks = 0.3f;  // Spring constant
  
  // Configure Tendon 1
  tendons[0].csPin = 10;
  tendons[0].encoderId = 0;
  tendons[0].forceType = ForceType::STEP;
  tendons[0].controller = new ForceControl(ff, kp, ki, kd, ks);
  
  // Configure Tendon 2
  tendons[1].csPin = 37;
  tendons[1].encoderId = 1;
  tendons[1].forceType = ForceType::SIN;
  tendons[1].controller = new ForceControl(ff, kp, ki, kd, ks);
  
  Serial.println("System initialized with the following configuration:");
  for (int i = 0; i < NUM_TENDONS; i++) {
    Serial.print("Tendon ");
    Serial.print(i + 1);
    Serial.print(": CS Pin = ");
    Serial.print(tendons[i].csPin);
    Serial.print(", Encoder ID = ");
    Serial.println(tendons[i].encoderId);
  }
}

void loop() {
  // Control and print data for each tendon
  for (int i = 0; i < NUM_TENDONS; i++) {
    Serial.print("Tendon ");
    Serial.print(i + 1);
    Serial.print(": ");
    
    // Run force PID control
    float force = tendons[i].controller->forcePID(
      tendons[i].csPin, 
      tendons[i].encoderId, 
      tendons[i].forceType
    );
    
    // Print force data
    tendons[i].controller->forcePrint();
  }
  
  delay(100); // Adjust control loop frequency as needed
}