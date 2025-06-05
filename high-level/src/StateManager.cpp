#include "StateManager.hpp"

StateManager::StateManager() 
    : wrist(std::make_unique<Wrist>()),
      dexFinger(std::make_unique<DexterousFinger>()),
      powFinger(std::make_unique<PowerFinger>()),
      kinematics(std::make_unique<TendonKinematics>()) {
    
}

bool StateManager::initialize() {
    Serial.println("Initializing StateManager.");
    currentMovementPhase = IDLE;
    return true;
}

bool StateManager::connectToMCU() {
    return true;
}

void StateManager::updateState(std::string& serialCommand) {
    if (checkSerial(serialCommand)) {
        Serial.printf("[HIGH] Received command: '%s'\n", serialCommand.c_str());
        
        uint8_t commandID;
        std::vector<float> values;
        
        if (parseSerialCommand(serialCommand, commandID, values)) {
            switch(commandID) {
                case SET_ALL_JOINTS:
                    this->setJointPositions(values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
                    break;
                default:
                    break;
            }
        } else {
            Serial.println("[HIGH] ERROR: Failed to parse command");
        }
        
        // Clear command buffer
        serialCommand.clear();
    }
}

void StateManager::updateGUI() {

}

void StateManager::controlLoop() {
    std::vector<double> dexControl = dexFinger->calculateControl();
    double powControl = powFinger->calculateControl();
    std::vector<double> wristControl = wrist->calculateControl();
    
    std::vector<double> motorTorques = kinematics->getMotorTorques({dexControl[0], dexControl[1], dexControl[2], dexControl[3], powControl, wristControl[0], wristControl[1]});
    Serial.print("[HIGH] INFO: Motor torques: ");
    Serial.print(motorTorques[0]);
    Serial.print(", ");
    Serial.print(motorTorques[1]);
    Serial.print(", ");
    Serial.print(motorTorques[2]);
    Serial.print(", ");
    Serial.print(motorTorques[3]);
    Serial.print(", ");
    Serial.print(motorTorques[4]);
    Serial.print(", ");
    Serial.print(motorTorques[5]);
    Serial.print(", ");
    Serial.println(motorTorques[6]);


    // For wrist
    motorTorques[0] = 0.0;
    motorTorques[1] = 0.0;
    motorTorques[2] = 0.0;
    motorTorques[3] = 0.0;
    motorTorques[4] = 0.0;
}

void StateManager::setJointPositions(double wristPitch, double wristYaw, double dexPip, double dexDip, double dexMcp, double dexSplay, double powGrasp) {
    dexFinger->setDexterousFingerPositions(dexPip, dexDip, dexMcp, dexSplay);
    powFinger->setJointAngles(powGrasp);
    wrist->setWristOrientation(wristPitch, wristYaw);
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

void StateManager::setMovementPhase(MovementPhase phase) {
    currentMovementPhase = phase;
}
