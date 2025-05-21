#include "odrive_can.hpp"
#include <Arduino.h>
#include <SPI.h>
#include "encoder.hpp"
#include "force_control.hpp"

// Pin definitions
const int ENCODER_SEA_CS = 10;  // Chip select pin for encoder
const int ENCODER_MOTOR_CS = 4;  // Chip select pin for encoder


// Constants
const long BAUD_RATE = 115200;
const int LOOP_TIME_MS = 10;  // 10ms control loop (100Hz)
const float CONSTANT_TORQUE = 0.004f;
const float TENSION_POS_THRES = 0.01f;

// Global variables
ForceControl forceController(0.0f, 0.0f, 0.0f, 0.0f, 1.0f); // Default values
unsigned long lastTime = 0;
unsigned long startTime = 0;
boolean runningPID = false;

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

// External CAN interface declared in odrive_can.cpp
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Implementation of onCanMessage that uses the global odrives array
void onCanMessage(const CAN_message_t& msg) {
    for (int i = 0; i < NUM_DRIVES; i++) {
        onReceive(msg, odrives[i].drive);
    }
}

// Helper function to check errors for all ODrives
void checkErrors() {
    for (int driveNum = 0; driveNum < NUM_DRIVES; driveNum++) {
        checkODriveErrors(odrives[driveNum].drive, odrives[driveNum].user_data);
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
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
        setupODrive(odrives[i].drive, odrives[i].user_data);
    }

    Serial.println("All ODrives ready!");

    // Initialize SPI for encoder
    SPI.begin();
    
    // Initialize force controller with encoder
    forceController.setMotorEncoder(ENCODER_MOTOR_CS);
    forceController.setSeaEncoder(ENCODER_SEA_CS);

    
    // Default to STEP reference
    forceController.setForceType(0);
    
    Serial.println("Force Control System Ready");
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
                    for (int i = 0; i < 100; i++) {
                        Serial.println(i);
                        delay(10); // Small delay between readings
                    }
                }
                break;
                
            case 9: // Stop PID
                runningPID = false;
                Serial.println("DATA_STREAM_STOPPED");
                break;
                
            default:
                runningPID = false;
                break;
        }
    }

    if (runningPID) {
        unsigned long currentTime = millis();
        
        // Run control loop at specified frequency (100Hz)
        if (currentTime - lastTime >= LOOP_TIME_MS) {
            // Calculate time in seconds for the reference signal
            unsigned long elapsedTime = currentTime - startTime;
            int timeSeconds = elapsedTime / 1000;  // Integer seconds for square wave
            
            // Update reference force (1N or 3N based on time)
            forceController.forceGeneration(forceController.getForceType(), timeSeconds);

            // Calculate control signal using PID controller
            float PIDtorque = forceController.forcePID(forceController.getForceType());
            
            // Apply torque to ODrive
            odrives[0].current_torque = PIDtorque;
            odrives[0].is_running = true;
            odrives[0].drive.setTorque(PIDtorque);
            
            // Update last execution time
            lastTime = currentTime;

            // Send timestamp (ms), reference force, and actual force
            Serial.print(elapsedTime);
            Serial.print(" ");
            Serial.print(forceController.getReferenceForce());
            Serial.print(" ");
            float loadCellReading = 0;
            Serial.println(loadCellReading);
        }
    }
    delay(1);  // Small delay to prevent overwhelming the system
}