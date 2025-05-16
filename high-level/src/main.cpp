#include <Arduino.h>
#include "StateManager.hpp"

void printSystemStatus();

// Create a state manager instance
StateManager stateManager("/dev/ttyACM0"); // Empty string as we don't need a specific port

// Timing variables
unsigned long lastCommandTime = 0;
const unsigned long commandInterval = 1000; // Send commands every 1 second

// Position value (will be incremented in the loop)
float positionValue = 0.0;
bool increasing = true;

void setup() {
    // Initialize serial for debugging
    Serial.begin(9600);
    delay(500); // Give some time for serial to initialize
    
    Serial.println("High-Level MCU starting...");
    
    // Initialize the StateManager (this will set up CAN communication)
    if (stateManager.initialize()) {
        Serial.println("StateManager initialized successfully");
    } else {
        Serial.println("Failed to initialize StateManager");
    }
}

void loop() {
    // Update the communication controller to process messages
    stateManager.controlLoop();
    
    // Send new position commands periodically
    if (millis() - lastCommandTime > commandInterval) {
        // Update the position value (0.0 to 45.0 degrees range)
        if (increasing) {
            positionValue += 5.0;
            if (positionValue >= 45.0) {
                positionValue = 45.0;
                increasing = false;
            }
        } else {
            positionValue -= 5.0;
            if (positionValue <= 0.0) {
                positionValue = 0.0;
                increasing = true;
            }
        }
        
        Serial.print("Setting new position values: ");
        Serial.println(positionValue);
        
        // Set the same position value for all joints as a simple demo
        stateManager.setJointPositions(
            positionValue, // wrist roll
            positionValue, // wrist pitch
            positionValue, // wrist yaw
            positionValue, // dexterous finger PIP
            positionValue, // dexterous finger DIP
            positionValue, // dexterous finger MCP
            positionValue, // dexterous finger splain
            positionValue  // power finger grasp
        );
        
        // Print current system status
        printSystemStatus();
        
        // Update timing
        lastCommandTime = millis();
    }
}

void printSystemStatus() {
    Serial.println("--- System Status ---");
    
    // Print current joint positions
    auto currentPositions = stateManager.getCurrentJointPositions();
    Serial.println("Current Positions:");
    for (const auto& pair : currentPositions) {
        Serial.print("  ");
        Serial.print(pair.first.c_str());
        Serial.print(": ");
        Serial.println(pair.second);
    }
    
    // Print desired joint positions
    auto desiredPositions = stateManager.getDesiredJointPositions();
    Serial.println("Desired Positions:");
    for (const auto& pair : desiredPositions) {
        Serial.print("  ");
        Serial.print(pair.first.c_str());
        Serial.print(": ");
        Serial.println(pair.second);
    }
    
    Serial.println("-------------------");
}