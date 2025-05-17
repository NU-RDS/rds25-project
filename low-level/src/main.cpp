// In low-level/src/main.cpp

#include "comms.hpp"
#include <Arduino.h>

// Define CAN driver
comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

// Define controller with low-level ID
comms::CommsController g_controller{
    g_canDriver,
    comms::MCUID::MCU_LOW_LEVEL_0 
};

// Timing for heartbeat
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000; // Send heartbeat every 1000ms

// Command buffer for handling received commands
comms::CommandBuffer g_commandBuffer;

// Motor control handler
class MotorControlHandler : public comms::CommandHandler {
public:
    void update(const comms::CommandMessagePayload &payload) override {
        // Extract motor control data
        uint32_t rawPayload = payload.payload;
        comms::MotorControlCommandOpt motorCmd;
        motorCmd.payload = rawPayload;
        
        // Get motor ID and control value
        uint8_t motorID = motorCmd.motorNumber;
        uint8_t value = motorCmd.value;
        
        // Apply motor control based on type
        switch (motorCmd.controlType) {
            case comms::MotorControlCommandType::MC_CMD_POS:
                // Set motor position
                setMotorPosition(motorID, value);
                break;
            case comms::MotorControlCommandType::MC_CMD_VEL:
                // Set motor velocity
                setMotorVelocity(motorID, value);
                break;
        }
    }
    
    bool isParallelizable(const std::vector<comms::CommandMessagePayload> slice) override {
        // Motor commands can run in parallel
        return true;
    }
    
private:
    void setMotorPosition(uint8_t motorID, uint8_t position) {
        // Implement motor position control
        Serial.printf("Setting motor %d position to %d\n", motorID, position);
        // This would connect to your motor driver code
    }
    
    void setMotorVelocity(uint8_t motorID, uint8_t velocity) {
        // Implement motor velocity control
        Serial.printf("Setting motor %d velocity to %d\n", motorID, velocity);
    }
};

// Send a heartbeat response to the high-level MCU
void sendHeartbeatResponse() {
    // Create a raw message with the appropriate ID for heartbeat response
    comms::RawCommsMessage message;
    
    // Get the message ID for heartbeat from this MCU
    comms::Option<uint32_t> idOpt = comms::MessageInfo::getMessageID(
        g_controller.me(), 
        comms::MessageContentType::MT_HEARTBEAT
    );
    
    if (idOpt.isNone()) {
        Serial.println("Error: Unable to get heartbeat message ID");
        return;
    }
    
    message.id = idOpt.value();
    
    // Payload can contain status information
    // For example, bits to indicate which motors are active, error states, etc.
    message.payload = 0x01; // Basic "alive" status
    
    // Send the message
    g_canDriver.sendMessage(message);
    
    Serial.println("Heartbeat sent");
}

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for serial port to connect
    }
    
    Serial.println("Low-Level MCU starting...");
    
    // Initialize controller
    g_controller.initialize();
    
    // Set up command handlers
    std::shared_ptr<MotorControlHandler> motorHandler = std::make_shared<MotorControlHandler>();
    g_commandBuffer.setHandler(comms::CommandType::CMD_MOTOR_CONTROL, motorHandler);
    
    // Initialize other hardware
    // ...
    
    Serial.println("Low-Level MCU initialized");
}

void loop() {
    // Update comms controller to process incoming messages
    g_controller.tick();
    
    // Update command buffer execution
    g_commandBuffer.tick();
    
    // Check if it's time to send a heartbeat
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        sendHeartbeatResponse();
        lastHeartbeatTime = currentTime;
    }
    
    // Other low-level tasks
    // ...
}