#include "odrive_can.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "encoder.hpp"
#include "force_control.hpp"

// Pin definitions
const int ENCODER_CS = 10;  // Chip select pin for encoder
const int ENCODER_ID = 0;   // ID for encoder

// Constants
const long BAUD_RATE = 115200;
const int LOOP_TIME_MS = 10;  // 10ms control loop (100Hz)

// Global variables
ForceControl forceController(0.0f, 0.0f, 0.0f, 0.0f, 1.0f); // Default values
unsigned long lastTime = 0;
boolean runningPID = false;

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Array of ODrives
const int NUM_DRIVES = 1;
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 0), ODriveUserData(), false, 0.0f},
};

// Constants
const float CONSTANT_TORQUE = 0.004f;
const float TENSION_POS_THRES = 0.01f;
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)

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


void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    
    Serial.println("Starting ODrive GUI Tensioning Demo");

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

    // Initialize SPI for encoder
    SPI.begin();
    
    // Initialize force controller with encoder
    forceController.setEncoder(ENCODER_CS, ENCODER_ID);
    
    // Default to STEP reference
    forceController.setForceType(0);
    
    Serial.println("Force Control System Ready");

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

void loop() {
    // Pump CAN events
    pumpEvents(can_intf);

    // Check for errors
    checkErrors();

    // Check for serial commands
  if (Serial.available() > 0) {
    int command = Serial.parseInt();
    Serial.read(); // Read the newline character
    
    switch (command) {
      case 0: // Set Feedforward term
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float ff = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setFf(ff);
        }
        break;
        
      case 1: // Set Kp
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float kp = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setKp(kp);
        }
        break;
        
      case 2: // Set Ki term
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float ki = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setKi(ki);
        }
        break;
        
      case 3: // Set Kd term
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          float kd = Serial.parseFloat();
          Serial.read(); // Read the newline character
          forceController.setKd(kd);
        }
        break;
        
      case 4: // Select reference type
        while (Serial.available() <= 0) {
          delay(10); // Wait for input
        }
        {
          int refType = Serial.parseInt();
          Serial.read(); // Read the newline character
          forceController.setForceType(refType);
        }
        break;

      case 5: // Run PID and plot
        runningPID = true;
        lastTime = millis(); // Reset timer
        break;
      
      case 6: // Show PID params
        {
          float ff = forceController.getFf();
          float kp = forceController.getKp();
          float ki = forceController.getKi();
          float kd = forceController.getKd();
          int forcetype = int(forceController.getForceType());
          Serial.println(ff);
          Serial.println(kp);
          Serial.println(ki);
          Serial.println(kd);
          Serial.println(forcetype);
        }
        break;

      case 7: // Show Encoder - read 100 values
        {
          Encoder encoder(ENCODER_CS, ENCODER_ID);
          for (int i = 0; i < 100; i++) {
            float angle = encoder.readEncoderDeg();
            Serial.println(angle);
            delay(10); // Small delay between readings
          }
        }
        break;
        
      case 8: // Quit
        runningPID = false;
        break;
        
      default:
        runningPID = false;
        break;
    }

  }

    delay(1);  // Small delay to prevent overwhelming the system
}