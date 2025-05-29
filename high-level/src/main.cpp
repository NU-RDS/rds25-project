#include <Arduino.h>
#include <string>

#include "StateManager.hpp"
#include "comms.hpp"

StateManager state_manager;

comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

comms::CommsController g_controller{
    g_canDriver,
    comms::MCUID::MCU_HIGH_LEVEL
};

std::string serialCommand = "";

bool checkSerial(std::string& serialCommand) {
    static bool commandReady = false;
    
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            commandReady = true;
        } else if (c != '\r') {
            serialCommand += c;
            if (serialCommand.length() > 100) {
                serialCommand.clear();
                Serial.println("[HIGH] ERROR: Command buffer overflow");
                return false;
            }
        }
    }
    
    if (commandReady) {
        commandReady = false;
        return true;  // Command is ready to be processed
    }
    
    return false;  // No complete command yet
}

void setup() {
    Serial.begin(9600);
    Serial.println("[HIGH]");
    g_controller.initialize();
    state_manager.initialize();
}

void loop() {
    if (checkSerial(serialCommand)) {
        // Parse the command
        uint8_t commandID = 0xFF;  // Default invalid
        float value = 0.0f;
        bool hasValue = false;
        
        // Simple parsing: "ID" or "ID:value"
        bool validCommand = false;
        size_t colonPos = serialCommand.find(':');
        
        if (colonPos != std::string::npos) {
            // Format: "ID:value"
            std::string idStr = serialCommand.substr(0, colonPos);
            std::string valueStr = serialCommand.substr(colonPos + 1);
            
            // Simple validation and conversion
            if (!idStr.empty() && !valueStr.empty()) {
                commandID = atoi(idStr.c_str());
                value = atof(valueStr.c_str());
                hasValue = true;
                validCommand = true;
            } else {
                Serial.println("[HIGH] Parse error: Invalid format");
            }
        } else {
            // Format: "ID" (no value)
            if (!serialCommand.empty()) {
                commandID = atoi(serialCommand.c_str());
                hasValue = false;
                validCommand = true;
            } else {
                Serial.println("[HIGH] Parse error: Empty command");
            }
        }
        
        if (validCommand) {
            Serial.print("[HIGH] Received - ID: ");
            Serial.print(commandID);
            if (hasValue) {
                Serial.print(" Value: ");
                Serial.print(value);
            }
            Serial.println();
            
            // TODO: Add your command handling logic here
            // Example:
            // switch (commandID) {
            //     case 1:
            //         // Handle command 1
            //         break;
            //     case 2:
            //         // Handle command 2
            //         break;
            //     default:
            //         Serial.println("[HIGH] Unknown command");
            //         break;
            // }
        }
        
        // Clear command buffer
        serialCommand.clear();
    }

    comms::Option<comms::CommsTickResult> tick_result = g_controller.tick();

    if (tick_result.isSome()) {
        comms::MessageInfo message_info = tick_result.value().info;
        comms::RawCommsMessage message_raw = tick_result.value().rawMessage;
        
        state_manager.processMessage(message_info, message_raw);
    }
    else {
        // DO something else if no feedback
    }

    MovementPhase phase = state_manager.getMovementPhase();

    if (phase == MovementPhase::IDLE) {

    }

    delay(1);
}