#include "PositionControl.hpp"

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