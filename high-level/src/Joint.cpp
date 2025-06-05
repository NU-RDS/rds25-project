#include "Joint.hpp"

Joint::Joint(const std::string& name, const double kp, const double kd) {
    this->name = name;
    this->currentPosition = 0.0;
    this->desiredPosition = 0.0;
    this->commandTorque = 0.0;
    this->maxLimit = M_PI;
    this->minLimit = 0.0;

    controller = std::make_unique<PositionControl>(kp, kd, 1.0);
}

const std::string& Joint::getName() {
    return this->name;
}

double Joint::getCurrentPosition() {
    return this->currentPosition;
}

double Joint::getDesiredPosition() {
    return this->desiredPosition;
}

double Joint::getControlSignal() {
    return this->controlSignal;
}

double Joint::getCommandTorque() {
    return this->commandTorque;
}

double Joint::getMotorValue() {
    return this->motorValue;
}

// Likely from encoder
void Joint::setCurrentPosition(double current_pos) {
    this->currentPosition = current_pos;
}

void Joint::setDesiredPosition(double desired_pos) {
    this->desiredPosition = desired_pos;
}

void Joint::setCommandTorque(double command_torque) {
    this->commandTorque = command_torque;
}

void Joint::setControlSignal(double control_signal) {
    this->controlSignal = control_signal;
}

void Joint::setMotorValue(double motor_value) {
    this->motorValue = motor_value;
}

double Joint:: calculateControlSignal() {
    controlSignal = controller->positionPD(desiredPosition, currentPosition);
    return controlSignal;
}