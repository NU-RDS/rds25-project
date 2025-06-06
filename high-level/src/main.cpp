#include <Arduino.h>
#include <string>

#include "ODriveCan.hpp"
#include "StateManager.hpp"

StateManager state_manager;
std::string serialCommand = "";

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

const int NUM_DRIVES = 3;
const int BUTTON_PIN = 24;

struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
    float encoder_offset;
    bool offset_captured;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 5), ODriveUserData(), false, 0.0f, 0.0f, false},
    {ODriveCAN(wrap_can_intf(can_intf), 6), ODriveUserData(), false, 0.0f, 0.0f, false},
    {ODriveCAN(wrap_can_intf(can_intf), 4), ODriveUserData(), false, 0.0f, 0.0f, false}
};

bool button_pressed = false;
bool last_button_state = false;
unsigned long last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce delay

// Button wait timing variables
unsigned long button_press_time = 0;
const unsigned long BUTTON_WAIT_DELAY = 5000; // 5 seconds in milliseconds
bool waiting_after_button_press = false;

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

    // Set control mode to torque control
    odrives[index].drive.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, 
                                         ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    Serial.print("Orive ");
    Serial.print(index);
    Serial.println(": controller mode set");

    // Enable closed loop control
    while (odrives[index].user_data.last_heartbeat.Axis_State != 
           ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        Serial.println("Cannot set closed loop");
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

    // Setup button pin
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    
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
    Serial.println("Odrive all set up");

    captureEncoderOffsets();

    state_manager.initialize();
    Serial.println("System ready");
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

void checkButton() {
    bool current_button_reading = digitalRead(BUTTON_PIN);
    
    // If the button state has changed, reset the debounce timer
    if (current_button_reading != last_button_state) {
        last_debounce_time = millis();
    }
    
    // If enough time has passed since the last state change
    if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
        // If the button state is stable and different from our stored state
        if (current_button_reading && !button_pressed) {
            button_pressed = true;
            button_press_time = millis(); // Record when button was pressed
            waiting_after_button_press = true;
            Serial.println("Button pressed - Starting 5 second wait...");
        }
    }
    
    last_button_state = current_button_reading;
}

void loop() {
    static unsigned long lastControlTime = 0;
    const unsigned long CONTROL_PERIOD_MS = 10; // 100Hz control rate

    pumpEvents(can_intf);
    checkButton();
    
    // Check if we're waiting after button press
    if (waiting_after_button_press) {
        if (millis() - button_press_time >= BUTTON_WAIT_DELAY) {
            waiting_after_button_press = false;
            Serial.println("5 second wait completed - PID control activated for ODrives 0 and 1");
        } else {
            // Still waiting - show countdown every second
            static unsigned long last_countdown = 0;
            if (millis() - last_countdown >= 1000) {
                last_countdown = millis();
                unsigned long remaining = (BUTTON_WAIT_DELAY - (millis() - button_press_time)) / 1000;
                Serial.print("Waiting... ");
                Serial.print(remaining);
                Serial.println(" seconds remaining");
                odrives[0].drive.setTorque(0.0f);
            }
            return; // Exit loop early while waiting
        }
    }
    
    if (millis() - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = millis();

        state_manager.updateState(serialCommand);

        Get_Encoder_Estimates_msg_t encoder = odrives[1].user_data.last_feedback;
        // motor ang is motor shaft ang
        float motor_pitch = state_manager.getKinematics()->toShaft(state_manager.getKinematics()->RevToDeg(encoder.Pos_Estimate - odrives[1].encoder_offset));
        state_manager.getWrist()->getPitch()->setMotorValue(motor_pitch);
        Serial.print("Motor ");
        Serial.print(1);
        Serial.print(" is at ");
        Serial.println(state_manager.getWrist()->getPitch()->getMotorValue());

        encoder = odrives[0].user_data.last_feedback;
        motor ang is motor shaft ang
        float motor_yaw = state_manager.getKinematics()->toShaft((state_manager.getKinematics()->RevToDeg(encoder.Pos_Estimate - odrives[0].encoder_offset)));
        state_manager.getWrist()->getYaw()->setMotorValue(motor_yaw);
        Serial.print("Motor ");
        Serial.print(0);
        Serial.print(" is at ");
        Serial.println(state_manager.getWrist()->getYaw()->getMotorValue());

        std::vector<float> current_joint_angles = state_manager.getKinematics()->motorToJointAngle(motor_pitch, motor_yaw);
        state_manager.getWrist()->getPitch()->setCurrentPosition(current_joint_angles[1]);
        state_manager.getWrist()->getYaw()->setCurrentPosition(current_joint_angles[0]);

        state_manager.controlLoop();

        // Control ODrive 1 (pitch) - PID when button pressed and wait is complete
        if (button_pressed && !waiting_after_button_press) {
            odrives[1].current_torque = 0.5f;
            odrives[1].is_running = true;
            float pitch_control = state_manager.getWrist()->getPitch()->getControlSignal();
            if (pitch_control > 2.0f) {
                pitch_control = 2.0f;
            }
            if (pitch_control < -2.0f) {
                pitch_control = -2.0f;
            }
            odrives[1].drive.setTorque(pitch_control);

            odrives[0].current_torque = 0.5f;
            odrives[0].is_running = true;
            float yaw_control = state_manager.getWrist()->getYaw()->getControlSignal();
            if (yaw_control > 2.0f) {
                yaw_control = 2.0f;
            }
            if (yaw_control < -2.0f) {
                yaw_control = -2.0f;
            }
            odrives[0].drive.setTorque(yaw_control);
            
        } else {
            odrives[2].drive.setTorque(0.9f);
            odrives[1].drive.setTorque(0.0f);
            odrives[0].drive.setTorque(0.0f);
        }        
    }
}