#include "Joint.hpp"

/**
 * @brief Constructs a Joint object with the specified name and PID controller gains.
 *
 * Initializes the joint's name, sets default values for position, torque, limits, and encoder offset.
 * Also creates a PositionControl controller with the provided PID gains and a default control period of 1.0.
 *
 * @param name The name of the joint.
 * @param kp Proportional gain for the position controller.
 * @param ki Integral gain for the position controller.
 * @param kd Derivative gain for the position controller.
 */
Joint::Joint(const std::string& name, const double kp, const double ki, const double kd) {
    this->name = name;
    this->currentPosition = 0.0;
    this->desiredPosition = 0.0;
    this->commandTorque = 0.0;
    this->maxLimit = M_PI;
    this->minLimit = 0.0;
    this->encoderOffset = 0.0;

    controller = std::make_unique<PositionControl>(kp, ki, kd, 1.0);
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

double Joint::getEncoderOffset() {
    return this->encoderOffset;
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

void Joint::setEncoderOffset(double encoder_offset) {
    this->encoderOffset = encoder_offset;
}

/**
 * @brief Calculates the control signal for the joint using a position PD controller.
 *
 * This function computes the control signal by invoking the positionPD method
 * of the associated controller, using the desired and current positions of the joint.
 * The computed control signal is stored and also returned.
 *
 * @return The calculated control signal as a double.
 */
double Joint:: calculateControlSignal() {
    controlSignal = controller->positionPD(desiredPosition, currentPosition);
    return controlSignal;
}