#include "StateManager.hpp"

StateManager::StateManager() 
    : wrist(std::make_unique<Wrist>()),
      dexFinger(std::make_unique<DexterousFinger>()),
      powFinger(std::make_unique<PowerFinger>()) {
    
}

bool StateManager::initialize() {
    Serial.println("Initializing StateManager.");
    currentMovementPhase = IDLE;
    return true;
}

bool StateManager::connectToMCU() {
    return true;
}

void StateManager::updateState() {

}

void StateManager::updateGUI() {

}

void StateManager::controlLoop() {
    // Update the communication controller to process messages
    
}

void StateManager::processMessage(const comms::MessageInfo& message_info, const comms::RawCommsMessage& message_raw) {
    if (message_info.type == comms::MessageContentType::MT_COMMAND) {
        Serial.println("[HIGH] INFO: Command received.");
        comms::Result<comms::CommandMessagePayload> cmdRes = comms::CommandMessagePayload::fromRaw(message_raw);

        if (cmdRes.isError()) {
            COMMS_DEBUG_PRINT_ERROR("Unable to handle command: %s\,", cmdRes.error);
            return;
        }

        comms::CommandMessagePayload cmd = cmdRes.value();

        if (cmd.commandID == comms::CommandType::CMD_SENSOR_TOGGLE) {
            // Update encoder values
        }
    }
    else if (message_info.type == comms::MessageContentType::MT_HEARTBEAT) {
        Serial.println("[HIGH] INFO: Heartbeat received.");
    }
    else if (message_info.type == comms::MessageContentType::MT_ERROR) {
        Serial.println("[HIGH] ERROR: Error received!");
    }
    else {
        Serial.println("[HIGH] ERROR: Received unknown message!");
    }
}

void StateManager::setJointPositions(double wristRoll, double wristPitch, double wristYaw, double dexPip, double dexDip, double dexMcp, double dexSplain, double powGrasp) {
    wrist->setWristOrientation(wristRoll, wristPitch, wristYaw);
    dexFinger->setDexterousFingerPositions(dexPip, dexDip, dexMcp, dexSplain);
    powFinger->setJointAngles(powGrasp);
}

std::unordered_map<std::string, double> StateManager::getCurrentJointPositions() {
    std::unordered_map<std::string, double> current_joints;
    

    return current_joints;
}

std::unordered_map<std::string, double> StateManager::getDesiredJointPositions() {
    std::unordered_map<std::string, double> desired_joints;
    

    return desired_joints;
}

void StateManager::executeSequencedMovement() {

}

void StateManager::stopMovement() {

}

void StateManager::pauseMovement() {

}

void StateManager::resumeMovement() {

}
