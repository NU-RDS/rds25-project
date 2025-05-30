#include "PowerFinger.hpp"

PowerFinger::PowerFinger() {
    Grasp = std::make_unique<Joint>("Grasp");
}

void PowerFinger::kinematics() {
    double joint_grasp = Grasp->getControlSignal();

    double motor_grasp = (J_pow * joint_grasp) / MOTOR_RADIUS;

    Grasp->setCommandTorque(motor_grasp);
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

double PowerFinger::calculateControl() {
    double grasp = Grasp->calculateControlSignal();
    return grasp;
}