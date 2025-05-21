#include <Arduino.h>

#include "StateManager.hpp"
#include "comms.hpp"

StateManager state_manager;

comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_500KBPS> g_canDriver;

comms::CommsController g_controller{
    g_canDriver,
    comms::MCUID::MCU_HIGH_LEVEL  // we are the high level
};

void setup() {
    Serial.begin(9600);
    Serial.println("[HIGH]");
    g_controller.initialize();
}

void loop() {
    Serial.println("Loop!");
    g_controller.tick();

    comms::MotorControlCommandOpt commandDesc(
        comms::MCUID::MCU_LOW_LEVEL_0,               // who is recieving it?
        0,                                    // what motor?
        comms::MotorControlCommandType::MC_CMD_POS,  // the control type
        10                                    // the value to control
    );

    comms::CommandMessagePayload motorCmd = comms::CommandBuilder::motorControl(g_controller.me(), commandDesc);

    g_controller.sendCommand(motorCmd);

    comms::SensorToggleCommandOpt toggleSensorDesc(
        comms::MCUID::MCU_LOW_LEVEL_0,  // who is recieving this?
        0,                       // what sensor?
        true                     // should it be outputing sensor values?
    );

    comms::CommandMessagePayload sensorCmd = comms::CommandBuilder::sensorToggle(g_controller.me(), toggleSensorDesc);

    g_controller.sendCommand(sensorCmd);

    delay(100);  // bad
}