#ifndef ODRIVE_CAN_HPP
#define ODRIVE_CAN_HPP

#include <Arduino.h>

// Include ODrive headers first to prevent naming conflicts
#include "ODriveCAN.h"
#include "ODriveEnums.h"
#include "ODriveFlexCAN.hpp"

// Then include FlexCAN_T4
#undef CAN_ERROR_BUS_OFF  // Undefine the conflicting macro
#include <FlexCAN_T4.h>

// Configuration
#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 0

// Forward declaration
struct ODriveStatus;

// User data structure
/**
 * @brief Stores user-related data for an ODrive device.
 *
 * This structure maintains the latest received messages and their status flags
 * for an ODrive device on the CAN bus. It tracks heartbeat, encoder feedback,
 * and current feedback messages, along with flags indicating whether each type
 * of message has been received.
 *
 * Members:
 * - last_heartbeat: The most recent heartbeat message received from the ODrive.
 * - received_heartbeat: True if a heartbeat message has been received.
 * - last_feedback: The most recent encoder estimates message received.
 * - received_feedback: True if an encoder feedback message has been received.
 * - last_current: The most recent current (Iq) message received.
 * - received_current: True if a current feedback message has been received.
 */
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    Get_Iq_msg_t last_current;
    bool received_current = false;
};



// Function declarations
bool setupCan();
void onCanMessage(const CAN_message_t& msg);
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
void getCurrent(Get_Iq_msg_t& msg, void* user_data);

#endif // ODRIVE_CAN_HPP