#include "DexterousFinger.hpp"

/**
 * @brief Constructs a DexterousFinger object and initializes its joints.
 *
 * This default constructor creates unique pointers for each of the finger's joints:
 * PIP (Proximal Interphalangeal), DIP (Distal Interphalangeal), MCP (Metacarpophalangeal),
 * and Splay. Each joint is initialized with default parameters for name, stiffness, damping,
 * and range of motion.
 */
DexterousFinger::DexterousFinger() {
    PIP = std::make_unique<Joint>("PIP", 1.0, 0.01, 1.0);
    DIP = std::make_unique<Joint>("DIP", 1.0, 0.01, 1.0);
    MCP = std::make_unique<Joint>("MCP", 1.0, 0.01, 1.0);
    Splay = std::make_unique<Joint>("Splay", 1.0, 0.01, 1.0);
}

/**
 * @brief Sets the desired positions for the finger joints.
 *
 * This function updates the target positions for the Proximal Interphalangeal (PIP),
 * Distal Interphalangeal (DIP), Metacarpophalangeal (MCP), and Splay joints of the dexterous finger.
 *
 * @param pip_desired   Desired position for the PIP joint.
 * @param dip_desired   Desired position for the DIP joint.
 * @param mcp_desired   Desired position for the MCP joint.
 * @param splay_desired Desired position for the Splay joint.
 */
void DexterousFinger::setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splay_desired) {
    PIP->setDesiredPosition(pip_desired);
    DIP->setDesiredPosition(dip_desired);
    MCP->setDesiredPosition(mcp_desired);
    Splay->setDesiredPosition(splay_desired);
}

/**
 * @brief Computes and sets the command torques for each joint of the dexterous finger based on current control signals.
 *
 * This method retrieves the current control signals from the Splay, PIP, DIP, and MCP joints.
 * It then calculates the required motor torques for each joint using the Jacobian matrix (J_dex),
 * a null-space vector (null_dex), and a scaling factor (MOTOR_RADIUS). The computed torques are
 * then set as command torques for the respective joints.
 *
 * @note Assumes that Splay, PIP, DIP, and MCP are valid joint objects with appropriate methods.
 * @note The null-space component (null_dex) is scaled by 0.1f to influence the torque calculation.
 */
void DexterousFinger::kinematics() {
    double joint_splay = Splay->getControlSignal();
    double joint_pip = PIP->getControlSignal();
    double joint_dip = DIP->getControlSignal();
    double joint_mcp = MCP->getControlSignal();

    double motor_splay = (J_dex[0][0] * joint_splay + J_dex[0][1] * joint_mcp + J_dex[0][2] * joint_pip + J_dex[0][3] * joint_dip + null_dex[0] * 0.1f) / MOTOR_RADIUS;
    double motor_pip = (J_dex[1][0] * joint_splay + J_dex[1][1] * joint_mcp + J_dex[1][2] * joint_pip + J_dex[1][3] * joint_dip + null_dex[1] * 0.1f) / MOTOR_RADIUS;
    double motor_mcp = (J_dex[2][0] * joint_splay + J_dex[2][1] * joint_mcp + J_dex[2][2] * joint_pip + J_dex[2][3] * joint_dip + null_dex[2] * 0.1f) / MOTOR_RADIUS;
    double motor_dip = (J_dex[3][0] * joint_splay + J_dex[3][1] * joint_mcp + J_dex[3][2] * joint_pip + J_dex[3][3] * joint_dip + null_dex[3] * 0.1f) / MOTOR_RADIUS;

    Splay->setCommandTorque(motor_splay);
    PIP->setCommandTorque(motor_pip);
    MCP->setCommandTorque(motor_mcp);
    DIP->setCommandTorque(motor_dip);
}

//void DexterousFinger::sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splay) {
//    // CAN library
//}

/**
 * @brief Retrieves the desired joint angles for the DexterousFinger.
 *
 * This method queries the desired positions for each of the finger's joints:
 * PIP (Proximal Interphalangeal), DIP (Distal Interphalangeal), MCP (Metacarpophalangeal),
 * and Splay. It returns a map associating each joint's name with its desired angle.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map containing the desired angles for each joint, keyed by joint name.
 */
const std::unordered_map<std::string, double> DexterousFinger::getDesiredJointAngles() {
    double pip = PIP->getDesiredPosition();
    double dip = DIP->getDesiredPosition();
    double mcp = MCP->getDesiredPosition();
    double splay = Splay->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"PIP", pip}, {"DIP", dip}, {"MCP", mcp}, {"Splay", splay}};

    return desired_joints;
}

/**
 * @brief Retrieves the current joint angles of the dexterous finger.
 *
 * This method queries the current positions of the PIP, DIP, MCP, and Splay joints,
 * and returns them as a mapping from joint names to their respective angles.
 *
 * @return std::unordered_map<std::string, double> 
 *         A map containing the current angles of the finger's joints, with keys:
 *         "PIP", "DIP", "MCP", and "Splay".
 */
const std::unordered_map<std::string, double> DexterousFinger::getCurrentJointAngles() {
    double pip = PIP->getCurrentPosition();
    double dip = DIP->getCurrentPosition();
    double mcp = MCP->getCurrentPosition();
    double splay = Splay->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"PIP", pip}, {"DIP", dip}, {"MCP", mcp}, {"Splay", splay}};

    return current_joints;
}

/**
 * @brief Calculates and returns the control signals for the finger joints.
 *
 * This method computes the control signals for the Splay, MCP (Metacarpophalangeal),
 * PIP (Proximal Interphalangeal), and DIP (Distal Interphalangeal) joints of the finger
 * by invoking their respective control signal calculation methods.
 *
 * @return std::vector<double> A vector containing the control signals in the following order:
 *         [Splay, MCP, PIP, DIP].
 */
std::vector<double> DexterousFinger::calculateControl() {
    double pip = PIP->calculateControlSignal();
    double dip = DIP->calculateControlSignal();
    double mcp = MCP->calculateControlSignal();
    double splay = Splay->calculateControlSignal();

    return {splay, mcp, pip, dip};
}