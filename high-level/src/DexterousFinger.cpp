#include "DexterousFinger.hpp"

DexterousFinger::DexterousFinger() {
    PIP = std::make_unique<Joint>("PIP");
    DIP = std::make_unique<Joint>("DIP");
    MCP = std::make_unique<Joint>("MCP");
    Splay = std::make_unique<Joint>("Splay");
}

void DexterousFinger::setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splay_desired) {
    PIP->setDesiredPosition(pip_desired);
    DIP->setDesiredPosition(dip_desired);
    MCP->setDesiredPosition(mcp_desired);
    Splay->setDesiredPosition(splay_desired);
}

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

void DexterousFinger::sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splain) {
    // CAN library
}

const std::unordered_map<std::string, double> DexterousFinger::getDesiredJointAngles() {
    double pip = PIP->getDesiredPosition();
    double dip = DIP->getDesiredPosition();
    double mcp = MCP->getDesiredPosition();
    double splain = Splay->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"PIP", pip}, {"DIP", dip}, {"MCP", mcp}, {"Splain", splain}};

    return desired_joints;
}

const std::unordered_map<std::string, double> DexterousFinger::getCurrentJointAngles() {
    double pip = PIP->getCurrentPosition();
    double dip = DIP->getCurrentPosition();
    double mcp = MCP->getCurrentPosition();
    double splay = Splay->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"PIP", pip}, {"DIP", dip}, {"MCP", mcp}, {"Splay", splay}};

    return current_joints;
}

std::vector<double> DexterousFinger::calculateControl() {
    double pip = PIP->calculateControlSignal();
    double dip = DIP->calculateControlSignal();
    double mcp = MCP->calculateControlSignal();
    double splay = Splay->calculateControlSignal();

    return {splay, mcp, pip, dip};
}