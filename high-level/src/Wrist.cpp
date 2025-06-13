#include "Wrist.hpp"

/**
 * @brief Constructs a Wrist object and initializes its joints.
 *
 * The constructor creates two unique Joint instances for the wrist:
 * - Pitch: Controls the pitch movement of the wrist, initialized with parameters (name: "Pitch", stiffness: 2.0, damping: 0.01, range: 0.5).
 * - Yaw: Controls the yaw movement of the wrist, initialized with parameters (name: "Yaw", stiffness: 2.0, damping: 0.01, range: 0.5).
 */
Wrist::Wrist() {
    Pitch = std::make_unique<Joint>("Pitch", 2.0, 0.01, 0.5);
    Yaw = std::make_unique<Joint>("Yaw", 2.0, 0.01, 0.5);
}

void Wrist::kinematics() {

}

/**
 * @brief Sets the desired orientation of the wrist by specifying pitch and yaw angles.
 *
 * This function updates the desired positions for both the pitch and yaw actuators
 * of the wrist mechanism. It should be called whenever a new wrist orientation is required.
 *
 * @param pitch_desired The desired pitch angle for the wrist.
 * @param yaw_desired The desired yaw angle for the wrist.
 */
void Wrist::setWristOrientation(double pitch_desired, double yaw_desired) {
    Pitch->setDesiredPosition(pitch_desired);
    Yaw->setDesiredPosition(yaw_desired);
}

void Wrist::sendTorqueCommand(double T_pitch, double T_yaw) {
    // Implement Evan's CAN communication??
}

/**
 * @brief Retrieves the desired joint angles for the wrist.
 *
 * This function queries the desired positions for both the Pitch and Yaw joints
 * of the wrist and returns them as a map, where the keys are the joint names ("Pitch" and "Yaw")
 * and the values are their respective desired angles.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map containing the desired angles for the "Pitch" and "Yaw" joints.
 */
const std::unordered_map<std::string, double> Wrist::getDesiredJointAngles() {
    double pitch = Pitch->getDesiredPosition();
    double yaw = Yaw->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"Pitch", pitch}, {"Yaw", yaw}};

    return desired_joints;
}

/**
 * @brief Retrieves the current joint angles of the wrist.
 *
 * This function queries the current positions of the Pitch and Yaw joints
 * and returns them as a map, where the keys are the joint names ("Pitch" and "Yaw")
 * and the values are their respective angles.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map containing the current angles of the Pitch and Yaw joints.
 */
const std::unordered_map<std::string, double> Wrist::getCurrentJointAngles() {
    double pitch = Pitch->getCurrentPosition();
    double yaw = Yaw->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"Pitch", pitch}, {"Yaw", yaw}};

    return current_joints;
}

/**
 * @brief Calculates the control signals for the wrist's pitch and yaw.
 *
 * This function computes the control signals for both the pitch and yaw joints
 * of the wrist by invoking their respective control signal calculation methods.
 * The resulting control signals are returned as a vector, where the first element
 * corresponds to the yaw control signal and the second to the pitch control signal.
 *
 * @return std::vector<double> A vector containing the yaw and pitch control signals, in that order.
 */
std::vector<double> Wrist::calculateControl() {
    double pitch = Pitch->calculateControlSignal();
    double yaw = Yaw->calculateControlSignal();

    return {yaw, pitch};
}