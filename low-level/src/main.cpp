// In low-level main.cpp or similar file
#include "comms.hpp"

// Define CAN driver
comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

// Define controller with low-level ID
comms::CommsController g_controller{
    g_canDriver,
    comms::MCUID::MCU_LOW_LEVEL_0  // First low-level MCU
};

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
        // This would connect to your motor driver code
    }
    
    void setMotorVelocity(uint8_t motorID, uint8_t velocity) {
        // Implement motor velocity control
    }
};

void setup() {
    Serial.begin(9600);
    
    // Initialize controller
    g_controller.initialize();
    
    // Set up command handlers
    std::shared_ptr<MotorControlHandler> motorHandler = std::make_shared<MotorControlHandler>();
    g_commandBuffer.setHandler(comms::CommandType::CMD_MOTOR_CONTROL, motorHandler);
    
    // Initialize other hardware
    // ...
}

void loop() {
    // Update comms controller
    g_controller.tick();
    
    // Other low-level tasks
    // ...
}