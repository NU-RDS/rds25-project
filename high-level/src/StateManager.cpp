#include "StateManager.hpp"

/**
 * @brief Constructs a StateManager object and initializes its components.
 *
 * This constructor initializes the StateManager by creating unique instances of
 * Wrist, DexterousFinger, PowerFinger, and TendonKinematics. These components are
 * managed using smart pointers to ensure proper memory management and ownership.
 */
StateManager::StateManager() 
    : wrist(std::make_unique<Wrist>()),
      dexFinger(std::make_unique<DexterousFinger>()),
      powFinger(std::make_unique<PowerFinger>()),
      kinematics(std::make_unique<TendonKinematics>()) {
    
}

/**
 * @brief Initializes the StateManager.
 *
 * This function sets the initial state of the StateManager by setting
 * the current movement phase to IDLE and prints an initialization message
 * to the serial output.
 *
 * @return true Always returns true to indicate successful initialization.
 */
bool StateManager::initialize() {
    Serial.println("Initializing StateManager.");
    currentMovementPhase = IDLE;
    return true;
}

bool StateManager::connectToMCU() {
    return true;
}

/**
 * @brief Processes and updates the state based on a received serial command.
 *
 * This function checks if a valid serial command has been received. If so, it attempts to parse the command,
 * extract the command ID and associated values, and execute the corresponding action. For example, if the command
 * is to set all joint positions, it converts the received values from degrees to radians and updates the joint positions.
 * If parsing fails, an error message is printed. After processing, the serial command buffer is cleared.
 *
 * @param serialCommand Reference to the string containing the received serial command. This buffer will be cleared after processing.
 */
void StateManager::updateState(std::string& serialCommand) {
    if (checkSerial(serialCommand)) {
        Serial.printf("[HIGH] Received command: '%s'\n", serialCommand.c_str());
        
        uint8_t commandID;
        std::vector<float> values;
        
        if (parseSerialCommand(serialCommand, commandID, values)) {
            switch(commandID) {
                case SET_ALL_JOINTS:
                    this->setJointPositions(kinematics->DegToRad(values[0]), kinematics->DegToRad(values[1]), kinematics->DegToRad(values[2]), kinematics->DegToRad(values[3]), kinematics->DegToRad(values[4]), kinematics->DegToRad(values[5]), kinematics->DegToRad(values[6]));
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

/**
 * @brief Executes the main control loop for the StateManager.
 *
 * This function calculates the control signals for the dexterous finger, power finger,
 * and wrist modules. It then computes the corresponding motor torques using the kinematics
 * module and the calculated control values. The resulting torques can be used to drive
 * the actuators of the system.
 *
 * The function assumes that the dexFinger, powFinger, wrist, and kinematics members are
 * properly initialized and that their respective calculateControl() and getMotorTorques()
 * methods return valid results.
 */
void StateManager::controlLoop() {
    std::vector<double> dexControl = dexFinger->calculateControl();
    double powControl = powFinger->calculateControl();
    std::vector<double> wristControl = wrist->calculateControl();
    
    std::vector<double> motorTorques = kinematics->getMotorTorques({dexControl[0], dexControl[1], dexControl[2], dexControl[3], powControl, wristControl[0], wristControl[1]});
    // Serial.print("[HIGH] INFO: Motor torques: ");
    // Serial.print(motorTorques[0]);
    // Serial.print(", ");
    // Serial.print(motorTorques[1]);
    // Serial.print(", ");
    // Serial.print(motorTorques[2]);
    // Serial.print(", ");
    // Serial.print(motorTorques[3]);
    // Serial.print(", ");
    // Serial.print(motorTorques[4]);
    // Serial.print(", ");
    // Serial.print(motorTorques[5]);
    // Serial.print(", ");
    // Serial.println(motorTorques[6]);
}

/**
 * @brief Sets the joint positions for the wrist, dexterous finger, and power finger.
 *
 * This function updates the positions and orientations of the hand's components:
 * - Sets the wrist orientation using the specified pitch and yaw angles.
 * - Sets the joint angles for the dexterous finger (PIP, DIP, MCP, and splay).
 * - Sets the joint angle for the power grasp.
 *
 * @param wristPitch The pitch angle for the wrist.
 * @param wristYaw The yaw angle for the wrist.
 * @param dexPip The PIP (proximal interphalangeal) joint angle for the dexterous finger.
 * @param dexDip The DIP (distal interphalangeal) joint angle for the dexterous finger.
 * @param dexMcp The MCP (metacarpophalangeal) joint angle for the dexterous finger.
 * @param dexSplay The splay angle for the dexterous finger.
 * @param powGrasp The joint angle for the power grasp.
 */
void StateManager::setJointPositions(double wristPitch, double wristYaw, double dexPip, double dexDip, double dexMcp, double dexSplay, double powGrasp) {
    dexFinger->setDexterousFingerPositions(dexPip, dexDip, dexMcp, dexSplay);
    powFinger->setJointAngles(powGrasp);
    wrist->setWristOrientation(wristPitch, wristYaw);
}

/**
 * @brief Retrieves the current positions of all joints.
 *
 * This function returns a map containing the names of the joints as keys
 * and their corresponding positions as values. The positions are represented
 * as double-precision floating point numbers.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map of joint names to their current positions.
 */
std::unordered_map<std::string, double> StateManager::getCurrentJointPositions() {
    std::unordered_map<std::string, double> current_joints;
    

    return current_joints;
}

/**
 * @brief Retrieves the desired positions for each joint.
 *
 * This function returns a mapping from joint names (as strings) to their desired
 * position values (as doubles). The desired positions represent the target
 * configuration for the joints, which can be used for motion planning or control.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map where each key is a joint name and each value is the desired position.
 */
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

/**
 * @brief Sets the current movement phase of the state manager.
 *
 * Updates the internal state to reflect the specified movement phase.
 *
 * @param phase The new movement phase to set.
 */
void StateManager::setMovementPhase(MovementPhase phase) {
    currentMovementPhase = phase;
}
