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
    _commsController.tick();
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