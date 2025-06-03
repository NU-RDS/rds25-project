#include <Arduino.h>
#include <SPI.h>
#include "odrive_can.hpp"
#include "Encoder.hpp"
#include "ForceControl.hpp"

#include "comms.hpp"

using namespace comms;

comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

comms::CommsController g_controller{
    g_canDriver,
    comms::MCUID::MCU_LOW_LEVEL_0
};

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
}

void loop() {
    g_controller.tick();
}