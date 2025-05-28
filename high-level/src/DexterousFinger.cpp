#include "DexterousFinger.hpp"

DexterousFinger::DexterousFinger() {
    PIP = std::make_unique<Joint>("PIP");
    DIP = std::make_unique<Joint>("DIP");
    MCP = std::make_unique<Joint>("MCP");
    Splain = std::make_unique<Joint>("Splain");
}

void DexterousFinger::kinematics() {
    // Need kinematic model
}

void DexterousFinger::setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splain_desired) {
    PIP->setDesiredPosition(pip_desired);
    DIP->setDesiredPosition(dip_desired);
    MCP->setDesiredPosition(mcp_desired);
    Splain->setDesiredPosition(splain_desired);
}

void DexterousFinger::sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splain) {
    // CAN library
}

const std::unordered_map<std::string, double> DexterousFinger::getDesiredJointAngles() {
    double pip = PIP->getDesiredPosition();
    double dip = DIP->getDesiredPosition();
    double mcp = MCP->getDesiredPosition();
    double splain = Splain->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"PIP", pip}, {"DIP", dip}, {"MCP", mcp}, {"Splain", splain}};

    return desired_joints;
}

const std::unordered_map<std::string, double> DexterousFinger::getCurrentJointAngles() {
    double pip = PIP->getCurrentPosition();
    double dip = DIP->getCurrentPosition();
    double mcp = MCP->getCurrentPosition();
    double splain = Splain->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"PIP", pip}, {"DIP", dip}, {"MCP", mcp}, {"Splain", splain}};

    return current_joints;
}