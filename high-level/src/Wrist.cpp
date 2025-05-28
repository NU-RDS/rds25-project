#include "Wrist.hpp"

Wrist::Wrist() {
    Roll = std::make_unique<Joint>("Roll");
    Pitch = std::make_unique<Joint>("Pitch");
    Yaw = std::make_unique<Joint>("Yaw");
}

void Wrist::kinematics() {

}

void Wrist::setWristOrientation(double roll_desired, double pitch_desired, double yaw_desired) {
    Roll->setDesiredPosition(roll_desired);
    Pitch->setDesiredPosition(pitch_desired);
    Yaw->setDesiredPosition(yaw_desired);
}

void Wrist::sendTorqueCommand(double T_roll, double T_pitch, double T_yaw) {
    // Implement Evan's CAN communication??
}

const std::unordered_map<std::string, double> Wrist::getDesiredJointAngles() {
    double roll = Roll->getDesiredPosition();
    double pitch = Pitch->getDesiredPosition();
    double yaw = Yaw->getDesiredPosition();

    std::unordered_map<std::string, double> desired_joints = {{"Roll", roll}, {"Pitch", pitch}, {"Yaw", yaw}};

    return desired_joints;
}

const std::unordered_map<std::string, double> Wrist::getCurrentJointAngles() {
    double roll = Roll->getCurrentPosition();
    double pitch = Pitch->getCurrentPosition();
    double yaw = Yaw->getCurrentPosition();

    std::unordered_map<std::string, double> current_joints = {{"Roll", roll}, {"Pitch", pitch}, {"Yaw", yaw}};

    return current_joints;
}