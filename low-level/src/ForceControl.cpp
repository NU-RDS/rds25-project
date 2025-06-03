#include "ForceControl.hpp"

ForceControl::ForceControl(float ff, float kp, float ki, float kd, float ks) :
    Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks), 
    referenceForce(0.0f), pidTorque(0.0f), 
    forceType(ForceType::STEP), seaEncoder(nullptr) {  // Initialize encoder pointer to nullptr
}

float ForceControl::encoderToTorque(float motor_angle, float sea_angle, int sea_id)  // Changed parameter to reference
{
    float diff = motor_angle - sea_angle;
    float force = diff * 0.00522f - 0.0000274 * diff * diff;

    return force;
}

// Implementation now matches declaration in header
float ForceControl::forcePID(float motor_angle, float sea_angle, float setTorque)
{
    static float prevErr = 0.0f;
    static float intErr = 0.0f;
    int sea_id = 0;

    // Get force from the encoders
    float SeaTorque = encoderToTorque(motor_angle, sea_angle, sea_id);
    
    // Error
    float error = setTorque - SeaTorque;
    intErr += error;
    
    // Anti-windup
    if (intErr > ANTI_WINDUP_F) {
        intErr = ANTI_WINDUP_F;
    } else if (intErr < -ANTI_WINDUP_F) {
        intErr = -ANTI_WINDUP_F;
    }
    
    // Derivative
    float dErr = error - prevErr;
    prevErr = error;
    
    // PID
    pidTorque = Ff * this->referenceForce + 
                     Kp * error + 
                     Ki * intErr + 
                     Kd * dErr;
    
    // Saturation limits
    if (pidTorque > MAX_TORQUE) {
        pidTorque = MAX_TORQUE;
    } else if (pidTorque < -MAX_TORQUE) {
        pidTorque = -MAX_TORQUE; 
    }
    
    return pidTorque;
}

void ForceControl::setForceType(int typeIndex)
{
    if (typeIndex == 0)
    {
        this->forceType = ForceType::STEP;
    }
    else if (typeIndex == 1)
    {
        this->forceType = ForceType::SIN;
    }
    else
    {
        this->forceType = ForceType::STEP;
    }
} 

float ForceControl::getSeaEncoderAngle(){
    if (this->seaEncoder == nullptr) {
        Serial.println("Error: Sea encoder not initialized");
        return 0.0f;
    }
    return this->seaEncoder->readEncoderDeg();
}

void ForceControl::setSeaEncoder(int encoderCS)
{
    if (this->seaEncoder != nullptr) {
        delete this->seaEncoder;  
    }
    this->seaEncoder = new Encoder(encoderCS);
}