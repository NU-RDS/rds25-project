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

enum CommandID {
    SET_JOINT_WRIST_ROLL = 1,
    SET_JOINT_WRIST_PITCH = 2,
    SET_JOINT_WRIST_YAW = 3,
    SET_JOINT_DEX_PIP = 4,
    SET_JOINT_DEX_DIP = 5,
    SET_JOINT_DEX_MCP = 6,
    SET_JOINT_DEX_SPLAIN = 7,
    SET_JOINT_POW_GRASP = 8,
    SET_ALL_JOINTS = 10,
    EMERGENCY_STOP = 20,
    EMERGENCY_CLEAR = 21,
    GET_STATUS = 30,
    GET_POSITIONS = 31,
    PRESET_OPEN_HAND = 40,
    PRESET_CLOSE_HAND = 41
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

std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token = "";
    
    for (char c : str) {
        if (c == delimiter) {
            if (!token.empty()) {
                tokens.push_back(token);
                token = "";
            }
        } else {
            token += c;
        }
    }
    
    if (!token.empty()) {
        tokens.push_back(token);
    }
    
    return tokens;
}

std::vector<float> parseFloatValues(const std::string& valueStr) {
    std::vector<float> values;
    std::vector<std::string> tokens = splitString(valueStr, ' ');
    
    for (const std::string& token : tokens) {
        if (!token.empty()) {
            float value = atof(token.c_str());
            values.push_back(value);
        }
    }
    
    return values;
}

bool parseSerialCommand(const std::string& command, uint8_t& commandID, std::vector<float>& values) {
    // Clear previous values
    values.clear();
    commandID = 0xFF; // Invalid default
    
    if (command.empty()) {
        return false;
    }
    
    // Find colon separator
    size_t colonPos = command.find(':');
    
    if (colonPos != std::string::npos) {
        // Format: "ID:value1 value2 value3..."
        std::string idStr = command.substr(0, colonPos);
        std::string valueStr = command.substr(colonPos + 1);
        
        if (!idStr.empty() && !valueStr.empty()) {
            commandID = atoi(idStr.c_str());
            values = parseFloatValues(valueStr);
            return true;
        }
    } else {
        // Format: "ID" (no values)
        commandID = atoi(command.c_str());
        return true;
    }
    
    return false;
}

void setup() {
    Serial.begin(9600);
    Serial.println("[HIGH]");
    g_controller.initialize();
    state_manager.initialize();
}

void loop() {
    if (checkSerial(serialCommand)) {
        Serial.printf("[HIGH] Received command: '%s'\n", serialCommand.c_str());
        
        uint8_t commandID;
        std::vector<float> values;
        
        if (parseSerialCommand(serialCommand, commandID, values)) {
            switch(commandID) {
                case SET_ALL_JOINTS:
                    state_manager.setJointPositions(values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
                    break;
                default:
                    break;
            }
        } else {
            Serial.println("[HIGH] ERROR: Failed to parse command");
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