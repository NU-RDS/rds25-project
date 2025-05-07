#include <Arduino.h>
#include <SPI.h>
#include "encoder.hpp"
#include "force_control.hpp"

// Pin definitions
const int ENCODER_CS = 10;  // Chip select pin for encoder
const int ENCODER_ID = 0;   // ID for encoder

// Constants
const long BAUD_RATE = 115200;
const int LOOP_TIME_MS = 10;  // 10ms control loop (100Hz)

// Global variables
ForceControl forceController(0.0f, 0.0f, 0.0f, 0.0f, 1.0f); // Default values
unsigned long lastTime = 0;
boolean runningPID = false;

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Initialize SPI for encoder
  SPI.begin();
  
  // Initialize force controller with encoder
  forceController.setEncoder(ENCODER_CS, ENCODER_ID);
  
  // Default to STEP reference
  forceController.setForceType(0);
  
  Serial.println("Force Control System Ready");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    int command = Serial.parseInt();
    Serial.read(); // Read the newline character
    
    switch (command) {
      case 0: // Set Feedforward term
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float ff = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setFf(ff);
        }
        break;
        
      case 1: // Set Kp
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float kp = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setKp(kp);
        }
        break;
        
      case 2: // Set Ki term
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float ki = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setKi(ki);
        }
        break;
        
      case 3: // Set Kd term
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float kd = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setKd(kd);
        }
        break;
        
      case 4: // Select reference type
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          int refType = Serial.parseInt();
          Serial.read(); // Read the newline character
          forceController.setForceType(refType);
        }
        break;

      case 5: // Run PID and plot
        // runningPID = true;
        lastTime = millis(); // Reset timer
        break;
      
      case 6: // Show PID params
        {
          float ff = forceController.getFf();
          float kp = forceController.getKp();
          float ki = forceController.getKi();
          float kd = forceController.getKd();
          int forcetype = int(forceController.getForceType());
          Serial.println(ff);
          Serial.println(kp);
          Serial.println(ki);
          Serial.println(kd);
          Serial.println(forcetype);
        }
        break;

      case 7: // Show Encoder - read 100 values
        {
          Encoder encoder(ENCODER_CS, ENCODER_ID);
          for (int i = 0; i < 100; i++) {
            float angle = encoder.readEncoderDeg();
            Serial.println(angle);
            delay(10); // Small delay between readings
          }
        }
        break;
        
      case 8: // Quit
        runningPID = false;
        break;
        
      default:
        runningPID = false;
        break;
    }
  }
  
  // If PID is running, execute control loop at specified interval
  if (runningPID) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime >= LOOP_TIME_MS) {
      lastTime = currentTime;
      
      // Call the PID function with all required arguments
      forceController.forcePID(ENCODER_CS, ENCODER_ID, forceController.getForceType());
      
      // Send reference and measured force data back to Python GUI
      Encoder encoder(ENCODER_CS, ENCODER_ID);
      float encoderForce = forceController.encoderToForce(encoder);
      
      // Send data back through serial
      Serial.print(forceController.getReferenceForce());
      Serial.print(" ");
      Serial.println(encoderForce);
    }
  }
}