#include <Arduino.h>
#include <string>

#include "ODriveCan.hpp"
#include "StateManager.hpp"

StateManager state_manager;
std::string serialCommand = "";

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

const int NUM_DRIVES = 4;
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
    float encoder_offset;
    bool offset_captured;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 0), ODriveUserData(), false, 0.0f, 0.0f, false},
    {ODriveCAN(wrap_can_intf(can_intf), 1), ODriveUserData(), false, 0.0f, 0.0f, false},
    {ODriveCAN(wrap_can_intf(can_intf), 2), ODriveUserData(), false, 0.0f, 0.0f, false},
    {ODriveCAN(wrap_can_intf(can_intf), 3), ODriveUserData(), false, 0.0f, 0.0f, false}
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
    while (!Serial) delay(100);

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

// Key fixes for main.cpp loop() function
void loop() {
    static unsigned long lastControlTime = 0;
    const unsigned long CONTROL_PERIOD_MS = 10; // 100Hz control rate

    pumpEvents(can_intf);
    
    if (millis() - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = millis();

        state_manager.updateState(serialCommand);

        // Read encoders and update motor positions
        Get_Encoder_Estimates_msg_t encoder = odrives[0].user_data.last_feedback;
        float motor_splay = state_manager.getKinematics()->toShaft(state_manager.getKinematics()->RevToRad(encoder.Pos_Estimate - odrives[0].encoder_offset));
        state_manager.getDexFinger()->getSplay()->setMotorValue(motor_splay);

        encoder = odrives[1].user_data.last_feedback;
        float motor_mcp = state_manager.getKinematics()->toShaft((state_manager.getKinematics()->RevToRad(encoder.Pos_Estimate - odrives[1].encoder_offset)));
        state_manager.getDexFinger()->getMCP()->setMotorValue(motor_mcp);

        encoder = odrives[2].user_data.last_feedback;
        float motor_pip = state_manager.getKinematics()->toShaft((state_manager.getKinematics()->RevToRad(encoder.Pos_Estimate - odrives[2].encoder_offset)));
        state_manager.getDexFinger()->getPIP()->setMotorValue(motor_pip);

        encoder = odrives[3].user_data.last_feedback;
        float motor_dip = state_manager.getKinematics()->toShaft((state_manager.getKinematics()->RevToRad(encoder.Pos_Estimate - odrives[3].encoder_offset)));
        state_manager.getDexFinger()->getDIP()->setMotorValue(motor_dip);

        // Calculate joint angles from motor positions
        std::vector<float> current_joint_angles = state_manager.getKinematics()->motorToJointAngleDex(motor_splay, motor_mcp, motor_pip, motor_dip);
        state_manager.getDexFinger()->getSplay()->setCurrentPosition(current_joint_angles[0]);
        state_manager.getDexFinger()->getMCP()->setCurrentPosition(current_joint_angles[1]);
        state_manager.getDexFinger()->getPIP()->setCurrentPosition(current_joint_angles[2]);
        state_manager.getDexFinger()->getDIP()->setCurrentPosition(current_joint_angles[3]);

        // Get current and desired positions for GUI feedback
        double splay_desired = state_manager.getDexFinger()->getSplay()->getDesiredPosition() * 180.0 / M_PI;
        double splay_current = state_manager.getDexFinger()->getSplay()->getCurrentPosition() * 180.0 / M_PI;
        double mcp_desired = state_manager.getDexFinger()->getMCP()->getDesiredPosition() * 180.0 / M_PI;
        double mcp_current = state_manager.getDexFinger()->getMCP()->getCurrentPosition() * 180.0 / M_PI;
        double pip_desired = state_manager.getDexFinger()->getPIP()->getDesiredPosition() * 180.0 / M_PI;
        double pip_current = state_manager.getDexFinger()->getPIP()->getCurrentPosition() * 180.0 / M_PI;
        double dip_desired = state_manager.getDexFinger()->getDIP()->getDesiredPosition() * 180.0 / M_PI;
        double dip_current = state_manager.getDexFinger()->getDIP()->getCurrentPosition() * 180.0 / M_PI;
        
        // Send formatted message for GUI (consolidated format)
        Serial.print("JOINT_ANGLES: splay_des=");
        Serial.print(splay_desired, 2);
        Serial.print(", splay_cur=");
        Serial.print(splay_current, 2);
        Serial.print(", mcp_des=");
        Serial.print(mcp_desired, 2);
        Serial.print(", mcp_cur=");
        Serial.print(mcp_current, 2);
        Serial.print(", pip_des=");
        Serial.print(pip_desired, 2);
        Serial.print(", pip_cur=");
        Serial.print(pip_current, 2);
        Serial.print(", dip_des=");
        Serial.print(dip_desired, 2);
        Serial.print(", dip_cur=");
        Serial.println(dip_current, 2);

        // Run control loop
        state_manager.controlLoop();
        
        // Apply control signals with safety limits
        float splay_control = state_manager.getDexFinger()->getSplay()->getControlSignal();
        splay_control = constrain(splay_control, -1.0f, 1.0f);  // Safety limit
        odrives[0].drive.setTorque(splay_control);
        odrives[0].current_torque = splay_control;
        odrives[0].is_running = true;

        float mcp_control = state_manager.getDexFinger()->getMCP()->getControlSignal();
        mcp_control = constrain(mcp_control, -1.0f, 1.0f);  // Safety limit
        odrives[1].drive.setTorque(mcp_control);
        odrives[1].current_torque = mcp_control;
        odrives[1].is_running = true;

        float pip_control = state_manager.getDexFinger()->getPIP()->getControlSignal();
        pip_control = constrain(pip_control, -1.0f, 1.0f);  // Safety limit
        odrives[2].drive.setTorque(pip_control);
        odrives[2].current_torque = pip_control;
        odrives[2].is_running = true;

        float dip_control = state_manager.getDexFinger()->getDIP()->getControlSignal();
        dip_control = constrain(dip_control, -1.0f, 1.0f);  // Safety limit
        odrives[3].drive.setTorque(dip_control);
        odrives[3].current_torque = dip_control;
        odrives[3].is_running = true;

        // Optional: Log control signals for debugging
        Serial.print("[DEBUG] Control signals - Splay: ");
        Serial.print(splay_control, 3);
        Serial.print(", MCP: ");
        Serial.print(mcp_control, 3);
        Serial.print(", PIP: ");
        Serial.print(pip_control, 3);
        Serial.print(", DIP: ");
        Serial.println(dip_control, 3);
    }
}