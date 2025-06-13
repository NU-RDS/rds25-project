#include "PositionControl.hpp"

/**
 * @brief Constructs a PositionControl object with specified PID and static gain parameters.
 *
 * Initializes the proportional (Kp), integral (Ki), derivative (Kd), and static (Ks) gains
 * for the position controller. Sets the default position control type to STEP and records
 * the current time as the previous time reference for control calculations.
 *
 * @param kp Proportional gain for the controller.
 * @param ki Integral gain for the controller.
 * @param kd Derivative gain for the controller.
 * @param ks Static gain for the controller.
 */
PositionControl::PositionControl(double kp, double ki, double kd, double ks) : Kp(kp), Ki(ki), Kd(kd), Ks(ks) {
    PositionControl::posType = PositionType::STEP;
    prevTime = std::chrono::steady_clock::now();
}

// NOT IMPLEMENTED 
void PositionControl::positionGeneration(PositionType posType, int t) {
    switch (posType) {
        case PositionType::STEP:
            break;
            
        case PositionType::SIN:
            break;

        case PositionType::MANUAL:
            break;
    }
}

/**
 * @brief Computes the control signal using a PID controller for position control.
 *
 * This function calculates the control output based on the difference between the desired
 * and current positions using proportional, integral, and derivative terms. It updates
 * the integral and derivative errors internally and applies an integral cap to prevent windup.
 *
 * @param desiredPosition The target position to achieve.
 * @param currentPosition The current measured position.
 * @return The computed control signal to drive the system towards the desired position.
 */
double PositionControl::positionPD(double desiredPosition, double currentPosition) {    
    auto currentTime = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(currentTime - prevTime).count();
    prevTime = currentTime;

    // Compute error
    double error = desiredPosition - currentPosition;

    if (integralError < integralCap) {
        integralError += error * dt;
    }
    else {
        integralError = integralCap;
    }

    // Compute derivative term
    double derivative = dt > 0.0 ? (error - prevError) / dt : 0.0;
    prevError = error;

    // Compute PID output
    double controlSignal = Kp * error + Ki * integralError + Kd * derivative;

    return controlSignal;
}