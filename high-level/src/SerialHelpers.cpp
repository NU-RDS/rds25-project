#include "SerialHelpers.hpp"

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
