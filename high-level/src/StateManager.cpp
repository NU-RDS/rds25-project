#include "StateManager.hpp"

StateManager::StateManager() {
    wrist = std::make_unique<Wrist>();
    dexFinger = std::make_unique<DexterousFinger>();
    powFinger = std::make_unique<PowerFinger>();

    currentMovementPhase = IDLE;
}

bool StateManager::initialize() {
    // Need to initialize connections to low level MCU
}

bool StateManager::connectToMCU(const std::string& port = "/dev/ttyUSB0") {

}

void StateManager::updateState() {

}

void StateManager::updateGUI() {

}

void StateManager::controlLoop() {

}

void StateManager::setJointPositions(double wristRoll, double wristPitch, double wristYaw, double dexPip, double dexDip, double dexMcp, double dexSplain, double powGrasp) {
    wrist->setWristOrientation(wristRoll, wristPitch, wristYaw);
    dexFinger->setDexterousFingerPositions(dexPip, dexDip, dexMcp, dexSplain);
    powFinger->setJointAngles(powGrasp);
}

std::unordered_map<std::string, double> StateManager::getCurrentJointPositions() {

}

std::unordered_map<std::string, double> StateManager::getDesiredJointPositions() {

}

void StateManager::executeSequencedMovement() {

}

void StateManager::stopMovement() {

}

void StateManager::pauseMovement() {

}

void StateManager::resumeMovement() {

}