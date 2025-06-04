#include <Arduino.h>
#include <SPI.h>
#include "odrive_can.hpp"
#include "Encoder.hpp"
#include "ForceControl.hpp"

#include "comms.hpp"

#define GEAR_REDUCTION 36.0f

using namespace comms;

// // comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

// comms::CommsController g_controller{
//     g_canDriver,
//     comms::MCUID::MCU_LOW_LEVEL_0
// };

// Global variables
float ks = 0.00103f;  // Spring constant in N/m
ForceControl forceController(0.1f, 0.1f, 0.0f, 0.0f, ks); // Default values

unsigned long lastTime = 0;
unsigned long startTime = 0;
float motor_offset = 0.0f;
float sea_offset = 0.0f;

void setup() {
    Serial.begin(115200);
    // Serial.println("[LOW]");
    // g_controller.initialize();

    while (!Serial);
    
    // Initialize ODrive array first (IMPORTANT: Add this line)
    initializeODrives();
    
    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }
    Serial.println("CAN initialized successfully");

    // Initialize all ODrives
    for (int i = 0; i < NUM_DRIVES; i++) {
        Serial.print("Initializing ODrive array index ");
        Serial.print(i);
        Serial.print(" (Node ID: ");
        Serial.print(odrive_idx[i]);
        Serial.println(")");
        setupODrive(i);
    }

    Serial.println("All ODrives ready!");

    // Initial pump to get heartbeats
    for (int i = 0; i < 50; i++) {
        pumpEvents(can_intf);
        delay(10);
    }
}

void loop() {
    // Pump CAN events
    pumpEvents(can_intf);

    // Check for errors
    checkErrors();

    // g_controller.tick();

    for (int odrive_index = 0; odrive_index < NUM_DRIVES; odrive_index++)
    {
        // Check if we have received heartbeat from this ODrive
        if (!odrives[odrive_index].user_data.received_heartbeat) {
            Serial.print("No heartbeat from ODrive ");
            Serial.print(odrive_index);
            Serial.print(" (Node ID: ");
            Serial.print(odrive_idx[odrive_index]);
            Serial.println(")");
            continue; // Skip this ODrive if no heartbeat
        }

        unsigned long currentTime = millis();
        
        // Check if we have feedback data
        if (odrives[odrive_index].user_data.received_feedback) {
            Get_Encoder_Estimates_msg_t feedback = odrives[odrive_index].user_data.last_feedback;
            Serial.print("Encoder ");
            Serial.print(odrive_index);
            Serial.print(" (Node ID: ");
            Serial.print(odrive_idx[odrive_index]);
            Serial.print(") -- revolutions: ");
            Serial.println(feedback.Pos_Estimate);
            
            // float motor_angle = feedback.Pos_Estimate * 360.0;
            // motor_angle = motor_angle/GEAR_REDUCTION - motor_offset;
            // float sea_angle = forceController.getSeaEncoderAngle() - sea_offset;
            // float PIDtorque = -forceController.forcePID(motor_angle, sea_angle, setTorque[odrive_index]);
        } else {
            Serial.print("No feedback from ODrive ");
            Serial.print(odrive_index);
            Serial.print(" (Node ID: ");
            Serial.print(odrive_idx[odrive_index]);
            Serial.println(")");
        }

        // Apply torque to ODrive
        float PIDtorque = 0.0f;
        odrives[odrive_index].current_torque = PIDtorque;
        odrives[odrive_index].is_running = true;
        odrives[odrive_index].drive.setTorque(PIDtorque);

        // Update last execution time
        lastTime = currentTime;
    }
    
    // Small delay to prevent overwhelming the CAN bus
    delay(1);
}