#include "StateManager.hpp"

StateManager::StateManager(const std::string& port) 
    : wrist(std::make_unique<Wrist>()),
      dexFinger(std::make_unique<DexterousFinger>()),
      powFinger(std::make_unique<PowerFinger>()),
      _commsController(_canDriver, comms::MCUID::MCU_HIGH_LEVEL) {
    
    currentMovementPhase = IDLE;
    mcuPort = port;
}

bool StateManager::initialize() {
    // Initialize CAN communication
    _commsController.initialize();
    
    // Other initialization
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
    _commsController.tick();
    
    // Check for received messages
    comms::RawCommsMessage message;
    
    // Use a non-blocking approach to check for messages
    if (_canDriver.receiveMessage(&message)) {
        // Get message info
        comms::Option<comms::MessageInfo> infoOpt = comms::MessageInfo::getInfo(message.id);
        
        if (infoOpt.isSome()) {
            comms::MessageInfo info = infoOpt.value();
            
            // Check if this is a heartbeat response
            if (info.type == comms::MessageContentType::MT_HEARTBEAT && 
                info.sender != comms::MCUID::MCU_HIGH_LEVEL) {
                
                // Call the heartbeat callback if set
                if (_heartbeatCallback) {
                    _heartbeatCallback(info.sender);
                }
            }
        }
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

void StateManager::sendHeartbeatRequest() {
    // Get the message ID for heartbeat request
    comms::Option<uint32_t> idOpt = comms::MessageInfo::getMessageID(
        _commsController.me(),
        comms::MessageContentType::MT_HEARTBEAT
    );
    
    if (idOpt.isNone()) {
        Serial.println("Error: Unable to get heartbeat request message ID");
        return;
    }
    
    comms::RawCommsMessage message;
    message.id = idOpt.value();
    message.payload = 0; // Empty payload for request
    
    // Send the message via the CAN driver
    _canDriver.sendMessage(message);
    
    Serial.println("Heartbeat request sent from StateManager");
}

void StateManager::setHeartbeatCallback(std::function<void(comms::MCUID)> callback) {
    // Store the callback for use when heartbeat responses are received
    // This requires adding a member variable to StateManager
    _heartbeatCallback = callback;
}