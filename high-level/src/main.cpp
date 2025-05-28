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
    state_manager.initialize();
}

void loop() {
    Serial.println("Loop!");
    comms::Option<comms::CommsTickResult> tick_result = g_controller.tick();

    if (tick_result.isSome()) {
        comms::MessageInfo message_info = tick_result.value().info;
        comms::RawCommsMessage message_raw = tick_result.value().rawMessage;
        
        state_manager.processMessage(message_info, message_raw);
    }
    else {
        // DO something else if no feedback
    }

    MovementPhase phase = state_manager.getMovementPhase();

    if (phase == MovementPhase::IDLE) {

    }
}