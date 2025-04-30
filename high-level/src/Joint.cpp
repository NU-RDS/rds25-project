#include "Joint.hpp"

Joint::Joint(const std::string& name) {
    this->name = name;
    this->currentPosition = 0.0;
    this->desiredPosition = 0.0;
    this->currentVelocity = 0.0;
    this->currentTorque = 0.0;
    this->commandTorque = 0.0;
    this->maxLimit = M_PI;
    this->minLimit = 0.0;
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

double Joint::getCurrentVelocity() {
    return this->currentVelocity;
}

double Joint::getCurrentTorque() {
    return this->currentTorque;
}

double Joint::getCommandTorque() {
    return this->commandTorque;
}

// Likely from encoder
void Joint::setCurrentPosition(double current_pos) {
    this->currentPosition = current_pos;
}

void Joint::setDesiredPosition(double desired_pos) {
    this->desiredPosition = desired_pos;
}

// From some computation or encoder
void Joint::setCurrentVelocity(double current_vel) {
    this->currentVelocity = current_vel;
}

void Joint::setCurrentTorque(double current_torque) {
    this->currentTorque = current_torque;
}

void Joint::setCommandTorque(double command_torque) {
    this->commandTorque = command_torque;
}