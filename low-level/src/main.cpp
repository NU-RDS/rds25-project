#include <Arduino.h>
#include <SPI.h>
#include "odrive_can.hpp"
#include "Encoder.hpp"
#include "ForceControl.hpp"

#include "comms.hpp"

#define GEAR_REDUCTION 36.0f

using namespace comms;

comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

comms::CommsController g_controller{
    g_canDriver,
    comms::MCUID::MCU_LOW_LEVEL_0
};

// Global variables
float ks = 0.00103f;  // Spring constant in N/m
ForceControl forceController(0.1f, 0.1f, 0.0f, 0.0f, ks); // Default values

unsigned long lastTime = 0;
unsigned long startTime = 0;
float motor_offset = 0.0f;
float sea_offset = 0.0f;

void setup() {
    Serial.begin(9600);
    Serial.println("[LOW]");
    g_controller.initialize();

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

    pumpEvents(can_intf);
    Get_Encoder_Estimates_msg_t feedback = odrives[0].user_data.last_feedback;

    float motor_angle = fmod(feedback.Pos_Estimate*360., 360.0);
    motor_offset = motor_angle/GEAR_REDUCTION;
    sea_offset = forceController.getSeaEncoderAngle();
}

void loop() {
    // Pump CAN events
    pumpEvents(can_intf);

    // Check for errors
    checkErrors();

    g_controller.tick();

    float setTorque[2] = {0, 0};

    for (int odrive_index = 0; odrive_index < NUM_DRIVES; odrive_index++)
    {
        unsigned long currentTime = millis();

        // Calculate control signal using PID controller
        odrives[odrive_index].is_running = true;
        Get_Encoder_Estimates_msg_t feedback = odrives[odrive_index].user_data.last_feedback;
        float motor_angle = feedback.Pos_Estimate * 360.0;
        motor_angle = motor_angle/GEAR_REDUCTION - motor_offset;
        float sea_angle = forceController.getSeaEncoderAngle() - sea_offset;
        float PIDtorque = -forceController.forcePID(motor_angle, sea_angle, setTorque[odrive_index]);

        // Apply torque to ODrive
        odrives[odrive_index].current_torque = PIDtorque;
        odrives[odrive_index].is_running = true;
        odrives[odrive_index].drive.setTorque(PIDtorque);

        // Update last execution time
        lastTime = currentTime;
    }
}