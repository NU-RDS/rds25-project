#include <Arduino.h>
#include <SPI.h>  // Ensure SPI library is included for SPI communication

#include <array>
#include <vector>

#include "Encoder.hpp"  // Include the header file for the Encoder class
#include "comms.hpp"

// Encoder chip selects and corresponding IDs
const std::array<int, 7> __csPins = {2, 3, 4, 5, 6, 14, 18};

comms::TeensyCANDriver<1, comms::CANBaudRate::CBR_250KBPS> commsDriver;
comms::CommsController controller{commsDriver, comms::MCUID::MCU_PALM};

void setup() {
    Serial.begin(115200);
    SPI.begin();  // Initialize SPI communication
    for (int i = 0; i < __csPins.size(); i++) {
        // setup pins
        pinMode(__csPins[i], OUTPUT);
        digitalWrite(__csPins[i], HIGH);  // Set CS pins high initially

        // add pins
        controller.addSensor(
            1, // interval in ms
            i, // sensor id
            std::make_shared<Encoder>(__csPins[i], i)); // sensor
    }

    controller.initialize();
}

void loop() {
    controller.tick();
}      