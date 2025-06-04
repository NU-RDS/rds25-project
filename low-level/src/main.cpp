#include "odrive_can.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "encoder.hpp"
#include "force_control.hpp"
#include "load_cell.hpp"

#define GEAR_REDUCTION 36.0f

// Pin definitions
const int ENCODER_SEA_CS = 10;  // Chip select pin for encoder
const int ENCODER_MOTOR_CS = 4;  // Chip select pin for encoder

const int32_t ZERO_OFFSET = -817;
const float NEWTONS_PER_COUNT = -0.000095;

// Constants
const long BAUD_RATE = 250000;  // CAN bus baud rate
const int LOOP_TIME_MS = 10;  // 10ms control loop (100Hz)

// Global variables
float ks = 0.00103f;  // Spring constant in N/m
ForceControl forceController(0.1f, 0.1f, 0.0f, 0.0f, ks); // Default values
unsigned long lastTime = 0;
unsigned long startTime = 0;
boolean runningPID = false;
float motor_offset = 0.0f;
float sea_offset = 0.0f;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// Array of ODrives
const int NUM_DRIVES = 1;
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 4), ODriveUserData(), false, 0.0f},
};

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

void onCanMessage(const CAN_message_t& msg) {
    for (int i = 0; i < NUM_DRIVES; i++) {
        onReceive(msg, odrives[i].drive);
    }
}

void setupODrive(int index) {
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
        Serial.println("Waiting for ODrive to enter closed loop control state...");
        odrives[index].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        Serial.println("Setting ODrive to closed loop control state");

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
}

void checkErrors(void) {
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


void setup() {
    //Serial.begin(BAUD_RATE);
    while (!Serial);
    
    // Initialize CAN
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

    Serial.println("All ODrives ready!");

    Serial.println("Force Control System Ready");
}

void loop() {
    // Pump CAN events
    pumpEvents(can_intf);

    // Check for errors
    checkErrors();

    odrives[0].is_running = true;
    Get_Encoder_Estimates_msg_t feedback = odrives[0].user_data.last_feedback;
    Serial.print(" - Heartbeat received: ");
    Serial.print(odrives[0].user_data.received_heartbeat ? "YES" : "NO");
    Serial.print(", Feedback received: ");
    Serial.println(odrives[0].user_data.received_feedback ? "YES" : "NO");

    odrives[0].current_torque = 0.5;
    odrives[0].is_running = true;
    odrives[0].drive.setTorque(0.5);

    delay(1);  // Small delay to prevent overwhelming the system
}