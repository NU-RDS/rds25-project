#include "odrive_can.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "encoder.hpp"
#include "force_control.hpp"
#include "load_cell.hpp"

#define GEAR_REDUCTION 36.0f
const int NUM_DRIVES = 5;

// Pin definitions
const int ENCODER_SEA_CS = 10;  // Chip select pin for encoder
const int ENCODER_MOTOR_CS = 4;  // Chip select pin for encoder

const int32_t ZERO_OFFSET = -643;
const float NEWTONS_PER_COUNT = -0.000093;

Loadcell loadcell(&Wire);
// Constants
const long BAUD_RATE = 250000;  // CAN bus baud rate
const int LOOP_TIME_MS = 10;  // 10ms control loop (100Hz)

// Global variables
float ks = 0.00103f;  // Spring constant in N/m
ForceControl forceController(0.0f, 0.1f, 0.0f, 0.0f, ks); // Default values
unsigned long lastTime = 0;
unsigned long startTime = 0;
boolean runningPID = false;
float motor_offset = 0.0f;
float sea_offset = 0.0f;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Array of ODrives
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 0), ODriveUserData(), false, 0.0f},
    {ODriveCAN(wrap_can_intf(can_intf), 1), ODriveUserData(), false, 0.0f},
    {ODriveCAN(wrap_can_intf(can_intf), 2), ODriveUserData(), false, 0.0f},
    {ODriveCAN(wrap_can_intf(can_intf), 3), ODriveUserData(), false, 0.0f},
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

    // Initialize SPI for encoder
    SPI.begin();
    
    // // Initialize force controller with encoder
    forceController.setSeaEncoder(ENCODER_SEA_CS);
        
    // Default to STEP reference
    forceController.setForceType(0);
    
    Get_Encoder_Estimates_msg_t feedback = odrives[0].user_data.last_feedback;

    float motor_angle = fmod(feedback.Pos_Estimate*360., 360.0);
    motor_offset = motor_angle/GEAR_REDUCTION;
    sea_offset = forceController.getSeaEncoderAngle();

    // Loadcell configuration
    loadcell.configure();
    loadcell.setNewtonsPerCount(NEWTONS_PER_COUNT);
    loadcell.setOffset(ZERO_OFFSET);

    Serial.println("Force Control System Ready");
}

void loop() {
    // Pump CAN events
    pumpEvents(can_intf);

    // Check for errors
    checkErrors();

    odrives[0].is_running = true;
    Get_Encoder_Estimates_msg_t feedback = odrives[0].user_data.last_feedback;
    Encoder encoder = Encoder(ENCODER_SEA_CS);
    //Serial.println(encoder.readEncoderDeg());

    // Serial.print(" - Revolutions: ");
    // Serial.print(feedback.Pos_Estimate);
    // Serial.print(" - Degrees: ");
    // Serial.println(fmod(feedback.Pos_Estimate*360., 360.0));
    

    // Check for serial commands
    if (Serial.available() > 0) 
    {
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
               startTime = millis(); // Set the absolute start time
               lastTime = startTime; // Reset timer for the control loop
               Serial.println("START_DATA_STREAM"); // Signal start of data
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
                   Encoder encoder = Encoder(ENCODER_MOTOR_CS);
                   for (int i = 0; i < 100; i++) {
                       Serial.print(i);
                       Serial.print(": ");
                       Serial.println(encoder.readEncoderDeg());
                       delay(10); // Small delay between readings
                   }
               }
               break;

           case 8: // Stop PID
               runningPID = false;
               Serial.println("DATA_STREAM_STOPPED");
               unsigned long startTime = millis();
               while (millis() - startTime < 1000)
               { 
                odrives[0].current_torque = 0.01;
                odrives[0].is_running = true;
                odrives[0].drive.setTorque(0.01);
               }
               break;
               
           default:
               runningPID = false;
               odrives[0].current_torque = 0;
               odrives[0].is_running = true;
               odrives[0].drive.setTorque(0);
               break;
       }
    } // <-- This closing brace was missing!

    if (runningPID)
    {        
        unsigned long currentTime = millis();

        // Calculate time in seconds for the reference signal
        unsigned long elapsedTime = currentTime - startTime;
        int timeSeconds = elapsedTime / 1000;  // Integer seconds for square wave

        // Update reference force (1N or 3N based on time)
        forceController.forceGeneration(forceController.getForceType(), timeSeconds);

        // Calculate control signal using PID controller
        odrives[0].is_running = true;
        Get_Encoder_Estimates_msg_t feedback = odrives[0].user_data.last_feedback;
        float motor_angle = feedback.Pos_Estimate * 360.0;
        motor_angle = motor_angle/GEAR_REDUCTION - motor_offset;
        float sea_angle = forceController.getSeaEncoderAngle() - sea_offset;
        float PIDtorque = -forceController.forcePID(motor_angle, sea_angle, forceController.getForceType());

        // Apply torque to ODrive
        //PIDtorque = .1;
        odrives[0].current_torque = PIDtorque;
        odrives[0].is_running = true;
        odrives[0].drive.setTorque(PIDtorque);
        // Serial.print("PID: ");
        // Serial.println(PIDtorque);

        // Update last execution time
        lastTime = currentTime;

        // Send timestamp (ms), reference force, and actual force
        Serial.print(elapsedTime);
        Serial.print(" ");
        Serial.print(forceController.getReferenceForce());
        Serial.print(" ");
        // float loadCellReading = 0;
        Serial.println(loadcell.readForce());

        // Serial.print("SEA angle: ");
        // Serial.println(forceController.getSeaEncoderAngle());
        // Serial.print("motor angle: ");
        // Serial.println(motor_angle);

        // Serial.println(forceController.encoderToForce(motor_angle, sea_angle));

    }
    
    delay(1);  // Small delay to prevent overwhelming the system
}