#include "PowerFinger.hpp"

PowerFinger::PowerFinger() {
    Grasp = std::make_unique<Joint>("Grasp");
}

void PowerFinger::kinematics() {

}

void PowerFinger::setJointAngles(double grasp_desired) {
    Grasp->setDesiredPosition(grasp_desired);
}

void PowerFinger::sendTorqueCommand(double T_grasp) {
    // Need CAN library
}

const std::unordered_map<std::string, double> PowerFinger::getDesiredJointAngles() {
    double grasp = Grasp->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"Grasp", grasp}};

    return desired_joints;
}

const std::unordered_map<std::string, double> PowerFinger::getCurrentJointAngles() {
    double grasp = Grasp->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"Grasp", grasp}};

    return current_joints;
}