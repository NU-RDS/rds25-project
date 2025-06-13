#include "SerialHelpers.hpp"

/**
 * @brief Reads characters from the serial input and assembles them into a command string.
 *
 * This function checks if data is available on the serial port. It reads characters one by one,
 * appending them to the provided serialCommand string until a newline character ('\n') is received,
 * which marks the end of a command. Carriage return characters ('\r') are ignored.
 * If the command buffer exceeds 100 characters, it is cleared and an error message is sent over serial.
 *
 * @param serialCommand Reference to a std::string where the incoming command is assembled.
 * @return true if a complete command (ending with '\n') has been received and is ready to be processed.
 * @return false if no complete command is available or if a buffer overflow occurred.
 */
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

/**
 * @brief Splits a string into a vector of substrings based on a specified delimiter character.
 *
 * This function iterates through the input string and separates it into tokens
 * whenever the delimiter character is encountered. Consecutive delimiters are treated
 * as separators, and empty tokens are not added to the result.
 *
 * @param str The input string to be split.
 * @param delimiter The character used to split the string.
 * @return std::vector<std::string> A vector containing the resulting substrings.
 */
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

/**
 * @brief Parses a space-separated string of float values into a vector of floats.
 *
 * This function takes a string containing float values separated by spaces,
 * splits the string into individual tokens, converts each token to a float,
 * and returns a vector containing the resulting float values.
 *
 * @param valueStr The input string containing space-separated float values.
 * @return std::vector<float> A vector of floats parsed from the input string.
 */
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

/**
 * @brief Parses a serial command string into a command ID and a list of float values.
 *
 * The command string can be in two formats:
 *   - "ID" (e.g., "3"): Only the command ID is provided, with no values.
 *   - "ID:value1 value2 ..." (e.g., "3:1.5 2.7 4.0"): The command ID is followed by a colon and a space-separated list of float values.
 *
 * @param[in]  command   The input command string to parse.
 * @param[out] commandID The parsed command ID. Set to 0xFF if parsing fails.
 * @param[out] values    The parsed list of float values. Cleared before parsing.
 * @return true if the command was successfully parsed; false otherwise.
 */
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
