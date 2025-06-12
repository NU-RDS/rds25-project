#ifndef SERIAL_HELPERS_HPP
#define SERIAL_HELPERS_HPP

#include <Arduino.h>
#include <string>

enum CommandID {
    SET_JOINT_WRIST_ROLL = 1,
    SET_JOINT_WRIST_PITCH = 2,
    SET_JOINT_WRIST_YAW = 3,
    SET_JOINT_DEX_PIP = 4,
    SET_JOINT_DEX_DIP = 5,
    SET_JOINT_DEX_MCP = 6,
    SET_JOINT_DEX_SPLAY = 7,
    SET_JOINT_POW_GRASP = 8,
    SET_ALL_JOINTS = 10,
    EMERGENCY_STOP = 20,
    EMERGENCY_CLEAR = 21,
    GET_STATUS = 30,
    GET_POSITIONS = 31,
    PRESET_OPEN_HAND = 40,
    PRESET_CLOSE_HAND = 41
};

bool checkSerial(std::string& serialCommand);
std::vector<std::string> splitString(const std::string& str, char delimiter);
std::vector<float> parseFloatValues(const std::string& valueStr);
bool parseSerialCommand(const std::string& command, uint8_t& commandID, std::vector<float>& values);

#endif // SERIAL_HELPER_HPP