#include <Arduino.h>
#include <string>

#include "ODriveCan.hpp"
#include "StateManager.hpp"

StateManager state_manager;
std::string serialCommand = "";

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

const int NUM_DRIVES = 2;
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
    float encoder_offset;
    bool offset_captured;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 0), ODriveUserData(), false, 0.0f, 0.0f, false},
    {ODriveCAN(wrap_can_intf(can_intf), 1), ODriveUserData(), false, 0.0f, 0.0f, false}
};

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
    // Register callbacks
    odrives[index].drive.onFeedback(onFeedback, &odrives[index].user_data);
    odrives[index].drive.onStatus(onHeartbeat, &odrives[index].user_data);
    // odrives[index].drive.getCurrent(getCurrents, &odrives[index].user_data);

    // Set control mode to torque control
    odrives[index].drive.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, 
                                         ODriveInputMode::INPUT_MODE_PASSTHROUGH);

    // Enable closed loop control
    while (odrives[index].user_data.last_heartbeat.Axis_State != 
           ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrives[index].drive.clearErrors();
        delay(1);
        odrives[index].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
}

void captureEncoderOffsets() {
    Serial.println("Capturing encoder offsets...");
    
    // Wait for initial encoder readings
    unsigned long timeout = millis() + 5000; // 5 second timeout
    
    for (int i = 0; i < NUM_DRIVES; i++) {
        odrives[i].offset_captured = false;
        
        while (!odrives[i].user_data.received_feedback && millis() < timeout) {
            pumpEvents(can_intf);
            delay(10);
        }
        
        if (odrives[i].user_data.received_feedback) {
            // Capture the initial encoder position as offset
            Get_Encoder_Estimates_msg_t encoder = odrives[i].user_data.last_feedback;
            odrives[i].encoder_offset = encoder.Pos_Estimate; // Store raw encoder value
            odrives[i].offset_captured = true;
            
            Serial.print("ODrive ");
            Serial.print(i);
            Serial.print(" offset captured: ");
            Serial.println(odrives[i].encoder_offset);
        } else {
            Serial.print("Failed to capture offset for ODrive ");
            Serial.println(i);
        }
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("[HIGH]");

    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    // Initialize all ODrives
    for (int i = 0; i < NUM_DRIVES; i++) {
        Serial.print("Initializing ODrive ");
        Serial.println(i);
        setupODrive(i);
    }

    captureEncoderOffsets();

    state_manager.initialize();
}

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

void onCanMessage(const CAN_message_t& msg) {
    for (int i = 0; i < NUM_DRIVES; i++) {
        onReceive(msg, odrives[i].drive);
    }
}

void loop() {
    pumpEvents(can_intf);

    state_manager.updateState(serialCommand);

    Get_Encoder_Estimates_msg_t encoder = odrives[0].user_data.last_feedback;
    // motor ang is motor shaft ang
    state_manager.getWrist()->getPitch()->setMotorValue(state_manager.getKinematics()->toShaft((state_manager.getKinematics()->RevToDeg(encoder.Pos_Estimate - odrives[0].encoder_offset))));
    Serial.print("Motor ");
    Serial.print(0);
    Serial.print(" is at ");
    Serial.println(state_manager.getWrist()->getPitch()->getMotorValue());

    encoder = odrives[1].user_data.last_feedback;
    // motor ang is motor shaft ang
    state_manager.getWrist()->getYaw()->setMotorValue(state_manager.getKinematics()->toShaft((state_manager.getKinematics()->RevToDeg(encoder.Pos_Estimate - odrives[1].encoder_offset))));
    Serial.print("Motor ");
    Serial.print(1);
    Serial.print(" is at ");
    Serial.println(state_manager.getWrist()->getYaw()->getMotorValue());

    // Need to convert from motor to joint

    state_manager.controlLoop();

    odrives[0].current_torque = 0.5f;
    odrives[0].is_running = true;
    odrives[0].drive.setTorque(state_manager.getWrist()->getPitch()->getControlSignal());

    odrives[1].current_torque = 0.5f;
    odrives[1].is_running = true;
    odrives[1].drive.setTorque(state_manager.getWrist()->getYaw()->getControlSignal());

    delay(10);
}