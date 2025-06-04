#ifndef ODRIVE_CAN_HPP
#define ODRIVE_CAN_HPP

#include <Arduino.h>
#include "TeensyID.hpp"

// Include ODrive headers first to prevent naming conflicts
#include "ODriveCAN.h"
#include "ODriveEnums.h"
#include "ODriveFlexCAN.hpp"

// Then include FlexCAN_T4
#undef CAN_ERROR_BUS_OFF  // Undefine the conflicting macro
#include <FlexCAN_T4.h>

// Configuration
#define CAN_BAUDRATE 250000
#define NUM_DRIVES 1
int odrive_idx[NUM_DRIVES] = {1}; // Your specific node IDs


// Forward declarations
struct ODriveUserData;
struct ODriveControl;
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

// User data structure
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    Get_Iq_msg_t last_current;
    bool received_current = false;
};

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// ODrive control structure
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
    
    // Constructor
    ODriveControl() : drive(ODriveCAN(wrap_can_intf(can_intf), 0)), 
                      user_data(), 
                      is_running(false), 
                      current_torque(0.0f) {}
    
    // Constructor with node ID
    ODriveControl(int node_id) : drive(ODriveCAN(wrap_can_intf(can_intf), node_id)), 
                                 user_data(), 
                                 is_running(false), 
                                 current_torque(0.0f) {}
};

// Global variables - declare after struct definitions
ODriveControl odrives[NUM_DRIVES];

// Initialize ODrive array properly
void initializeODrives() {
    for (int i = 0; i < NUM_DRIVES; i++) {
        odrives[i] = ODriveControl(odrive_idx[i]);
    }
}

// Callback implementations
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
}

void checkErrors() {
    for (int driveNum = 0; driveNum < NUM_DRIVES; driveNum++){
        // Check for errors and print them out
        Heartbeat_msg_t heartbeat = odrives[driveNum].user_data.last_heartbeat;
        if (heartbeat.Axis_Error != 0){
            Get_Error_msg_t msg;
            uint16_t timeout_ms = 50000;
            if (odrives[driveNum].drive.getError(msg, timeout_ms)) {
                Serial.print("Error: ");
                Serial.print(msg.Disarm_Reason);
                Serial.print(" ");
                Serial.println(msg.Active_Errors);
                odrives[driveNum].drive.clearErrors();
                odrives[driveNum].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        }
        delay(1);
    }
}

void onCanMessage(const CAN_message_t& msg) {
    // Extract node ID from CAN message ID
    uint32_t node_id = (msg.id >> 5) & 0x3F; // Extract bits 5-10 for ODrive node ID
    
    // Find which ODrive array index corresponds to this node ID
    for (int i = 0; i < NUM_DRIVES; i++) {
        if (odrive_idx[i] == node_id) {
            onReceive(msg, odrives[i].drive);
            break;
        }
    }
}

// CAN setup implementation
bool setupCan() {
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}

void setupODrive(int index) {
    // Bounds check
    if (index >= NUM_DRIVES) {
        Serial.print("Error: ODrive index ");
        Serial.print(index);
        Serial.print(" exceeds NUM_DRIVES (");
        Serial.print(NUM_DRIVES);
        Serial.println(")");
        return;
    }
    
    Serial.print("Setting up ODrive with node ID: ");
    Serial.println(odrive_idx[index]);
    
    // Register callbacks
    odrives[index].drive.onFeedback(onFeedback, &odrives[index].user_data);
    Serial.println("Registering heartbeat callback");
    odrives[index].drive.onStatus(onHeartbeat, &odrives[index].user_data);
    Serial.println("Registering feedback callback");

    // Set control mode to torque control
    odrives[index].drive.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, 
                                         ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    Serial.println("Setting control mode to torque control");

    // Enable closed loop control
    while (odrives[index].user_data.last_heartbeat.Axis_State != 
           ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrives[index].drive.clearErrors();
        delay(1);
        Serial.print("Waiting for ODrive ");
        Serial.print(odrive_idx[index]);
        Serial.println(" to enter closed loop control state...");
        odrives[index].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        Serial.print("Setting ODrive ");
        Serial.print(odrive_idx[index]);
        Serial.println(" to closed loop control state");

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
}

#endif // ODRIVE_CAN_HPP