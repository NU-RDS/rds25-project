#include <Arduino.h>
#include "StateManager.hpp"

void printSystemStatus();
void handleHeartbeatResponse(comms::MCUID sender);

// Create a state manager instance
StateManager stateManager("/dev/ttyACM0");

// Timing variables
unsigned long lastCommandTime = 0;
const unsigned long commandInterval = 1000; // Send commands every 1 second

unsigned long lastHeartbeatRequestTime = 0;
const unsigned long HEARTBEAT_REQUEST_INTERVAL = 2000; // Send heartbeat request every 2 seconds

// Track connected MCUs by monitoring heartbeat responses
bool lowLevelMCUsConnected[4] = {false, false, false, false}; // For LOW_LEVEL_0 through LOW_LEVEL_3
unsigned long lastHeartbeatTimes[4] = {0, 0, 0, 0}; // Last time a heartbeat was received from each MCU

// Position value (will be incremented in the loop)
float positionValue = 0.0;
bool increasing = true;

// This function is called when a heartbeat response is received
void handleHeartbeatResponse(comms::MCUID sender) {
    // Update the connection status for this MCU
    if (sender >= comms::MCUID::MCU_LOW_LEVEL_0 && 
        sender <= comms::MCUID::MCU_LOW_LEVEL_3) {
        int index = sender - comms::MCUID::MCU_LOW_LEVEL_0;
        lowLevelMCUsConnected[index] = true;
        lastHeartbeatTimes[index] = millis(); // Update the last heartbeat time
        
        Serial.printf("Received heartbeat from LOW_LEVEL_%d\n", index);
    }
}

// Check connection status of all MCUs
void monitorConnections() {
    unsigned long currentTime = millis();
    const unsigned long CONNECTION_TIMEOUT = 5000; // 5 seconds timeout
    
    // Check for timeouts
    for (int i = 0; i < 4; i++) {
        if (lowLevelMCUsConnected[i] && (currentTime - lastHeartbeatTimes[i] > CONNECTION_TIMEOUT)) {
            lowLevelMCUsConnected[i] = false;
            Serial.printf("LOW_LEVEL_%d disconnected (timeout)\n", i);
        }
    }
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(9600);
    delay(500); // Give some time for serial to initialize
    
    Serial.println("High-Level MCU starting...");
    
    // Initialize the StateManager (this will set up CAN communication)
    if (stateManager.initialize()) {
        Serial.println("StateManager initialized successfully");
        
        // Set the heartbeat callback if your StateManager supports it
        // stateManager.setHeartbeatCallback(handleHeartbeatResponse);
    } else {
        Serial.println("Failed to initialize StateManager");
    }
}

void loop() {
    // Update the communication controller to process messages
    stateManager.controlLoop();
    
    // Current time for timing operations
    unsigned long currentTime = millis();
    
    // Send heartbeat requests periodically
    if (currentTime - lastHeartbeatRequestTime >= HEARTBEAT_REQUEST_INTERVAL) {
        Serial.println("Sending heartbeat request to low-level MCUs...");
        
        // Call the sendHeartbeatRequest method from StateManager
        // If this method is not implemented yet, you'll need to add it
        stateManager.sendHeartbeatRequest();
        
        lastHeartbeatRequestTime = currentTime;
    }
    
    // Monitor connections for timeouts
    monitorConnections();
    
    // Send new position commands periodically
    if (currentTime - lastCommandTime > commandInterval) {
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
        lastCommandTime = currentTime;
    }
}

void printSystemStatus() {
    Serial.println("--- System Status ---");
    
    // Print connection status
    Serial.println("MCU Connection Status:");
    for (int i = 0; i < 4; i++) {
        Serial.printf("  LOW_LEVEL_%d: %s\n", i, lowLevelMCUsConnected[i] ? "Connected" : "Disconnected");
    }
    
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