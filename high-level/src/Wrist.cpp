#include "Wrist.hpp"

Wrist::Wrist() {
    Pitch = std::make_unique<Joint>("Pitch", 2.0, 0.0, 0.0);
    Yaw = std::make_unique<Joint>("Yaw", 2.0, 0.0, 0.0);
}

void Wrist::kinematics() {

}

void Wrist::setWristOrientation(double pitch_desired, double yaw_desired) {
    Pitch->setDesiredPosition(pitch_desired);
    Yaw->setDesiredPosition(yaw_desired);
}

void Wrist::sendTorqueCommand(double T_pitch, double T_yaw) {
    // Implement Evan's CAN communication??
}

const std::unordered_map<std::string, double> Wrist::getDesiredJointAngles() {
    double pitch = Pitch->getDesiredPosition();
    double yaw = Yaw->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"Pitch", pitch}, {"Yaw", yaw}};

    return desired_joints;
}

const std::unordered_map<std::string, double> Wrist::getCurrentJointAngles() {
    double pitch = Pitch->getCurrentPosition();
    double yaw = Yaw->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"Pitch", pitch}, {"Yaw", yaw}};

    return current_joints;
}

std::vector<double> Wrist::calculateControl() {
    double pitch = Pitch->calculateControlSignal();
    double yaw = Yaw->calculateControlSignal();

    return {yaw, pitch};
}