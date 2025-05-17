#include <Arduino.h>

#include "comms.hpp"

using namespace comms;

TeensyCANDriver<2, CANBaudRate::CBR_500KBPS> g_canDriver;

CommsController g_controller{
    g_canDriver,
    MCUID::MCU_LOW_LEVEL_0 // we are the high level
};

void setup() {
    g_controller.initialize();
}

void loop() {
    g_controller.tick();
}