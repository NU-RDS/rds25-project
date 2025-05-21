#include "odrive_can.hpp"

// Global CAN interface
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

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

void getCurrent(Get_Iq_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_current = msg;
    odrv_user_data->received_current = true;
}

void onCanMessage(const CAN_message_t& msg) {
    // This function will be defined in the main file since it depends on the odrives array
    // However, we need to declare it here to match the function prototype in the header
    // The actual implementation will forward the message to all ODrives
}

// Helper function to configure an ODrive
void setupODrive(ODriveCAN& drive, ODriveUserData& user_data) {
    // Register callbacks
    drive.onFeedback(onFeedback, &user_data);
    drive.onStatus(onHeartbeat, &user_data);
    // drive.getCurrent(getCurrent, &user_data);

    // Set control mode to torque control
    drive.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, 
                            ODriveInputMode::INPUT_MODE_PASSTHROUGH);

    // Enable closed loop control
    while (user_data.last_heartbeat.Axis_State != 
           ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        drive.clearErrors();
        delay(1);
        drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
}

// Check for ODrive errors and handle them
void checkODriveErrors(ODriveCAN& drive, ODriveUserData& user_data) {
    // Check for errors and print them out
    Heartbeat_msg_t heartbeat = user_data.last_heartbeat;
    if (heartbeat.Axis_Error != 0) {
        Get_Error_msg_t msg;
        uint16_t timeout_ms = 50000;
        if (drive.getError(msg, timeout_ms)) {
            Serial.print("Error: ");
            Serial.print(msg.Disarm_Reason);
            Serial.print(" ");
            Serial.println(msg.Active_Errors);
            drive.clearErrors();
            drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
    }
    delay(1);
}

// // Wrapper for CanIntf access
// CanIntf wrap_can_intf(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& can) {
//     return CanIntf(can);
// }