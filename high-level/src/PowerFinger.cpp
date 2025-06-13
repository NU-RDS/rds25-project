#include "PowerFinger.hpp"

/**
 * @brief Constructs a PowerFinger object and initializes the Grasp joint.
 *
 * This constructor creates a new instance of the PowerFinger class and
 * initializes the Grasp member with a unique pointer to a Joint object.
 * The Joint is named "Grasp" and is initialized with the parameters:
 * - position: 1.0
 * - velocity: 0.01
 * - effort: 1.0
 */
PowerFinger::PowerFinger() {
    Grasp = std::make_unique<Joint>("Grasp", 1.0, 0.01, 1.0);
}

/**
 * @brief Computes and applies the motor torque required for the finger's grasp action.
 *
 * This method retrieves the current control signal for the finger's grasp joint,
 * calculates the corresponding motor torque using the joint-to-motor transmission ratio
 * and the motor radius, and then sets this torque as the command for the grasp actuator.
 *
 * The calculation is as follows:
 *   - Retrieves the joint control signal from the Grasp actuator.
 *   - Computes the motor torque using the formula: (J_pow * joint_grasp) / MOTOR_RADIUS.
 *   - Sets the computed torque as the command torque for the Grasp actuator.
 */
void PowerFinger::kinematics() {
    double joint_grasp = Grasp->getControlSignal();

    double motor_grasp = (J_pow * joint_grasp) / MOTOR_RADIUS;

    Grasp->setCommandTorque(motor_grasp);
}

/**
 * @brief Sets the desired joint angles for the PowerFinger.
 *
 * This function updates the desired position of the finger's grasp mechanism
 * by passing the specified grasp angle to the underlying Grasp controller.
 *
 * @param grasp_desired The target angle (in radians or degrees, depending on implementation)
 *                      for the finger's grasp.
 */
void PowerFinger::setJointAngles(double grasp_desired) {
    Grasp->setDesiredPosition(grasp_desired);
}

void PowerFinger::sendTorqueCommand(double T_grasp) {
    // Need CAN library
}

/**
 * @brief Retrieves the desired joint angles for the PowerFinger.
 *
 * This function queries the current desired position for the "Grasp" joint
 * from the Grasp object and returns it in an unordered map, where the key
 * is the joint name ("Grasp") and the value is the corresponding desired angle.
 *
 * @return An unordered_map mapping joint names to their desired angles.
 */
const std::unordered_map<std::string, double> PowerFinger::getDesiredJointAngles() {
    double grasp = Grasp->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"Grasp", grasp}};

    return desired_joints;
}

/**
 * @brief Retrieves the current joint angles of the PowerFinger.
 *
 * This function queries the current position of the "Grasp" joint and returns
 * a map containing the joint name as the key and its corresponding angle as the value.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map with joint names as keys and their current angles as values.
 */
const std::unordered_map<std::string, double> PowerFinger::getCurrentJointAngles() {
    double grasp = Grasp->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"Grasp", grasp}};

    return current_joints;
}

/**
 * @brief Calculates the control signal for the PowerFinger.
 *
 * This function computes the control signal required for the PowerFinger
 * by delegating the calculation to the associated Grasp object.
 *
 * @return The computed control signal as a double.
 */
double PowerFinger::calculateControl() {
    double grasp = Grasp->calculateControlSignal();
    return grasp;
}