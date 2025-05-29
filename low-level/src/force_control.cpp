#include "force_control.hpp"

ForceControl::ForceControl(float ff, float kp, float ki, float kd, float ks) :
    Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks), 
    referenceForce(0.0f), pidCurrent(0.0f), 
    forceType(ForceType::STEP), seaEncoder(nullptr) {  // Initialize encoder pointer to nullptr
}

float ForceControl::encoderToForce(float motor_angle, Encoder& seaEncoder)  // Changed parameter to reference
{
    motor_angle = 0;//motorEncoder.readEncoderDeg();

    float sea_angle = 0;//seaEncoder.readEncoderDeg();

    float distance = R_ENCODER_PULLEY * ((motor_angle - sea_angle)); 

    return distance * this->Ks;
}

void ForceControl::forceGeneration(ForceType forceType, int t) {
    switch (forceType) {
        case ForceType::STEP:
            // Step input
            this->referenceForce = ((t/500) % 2 == 0) ? 1.0f : 3.0f;
            break;
            
        case ForceType::SIN:
            // Sinusoidal force (2 + 1 sin(2π f t))N
            {
                float frequency = 1.0f;
                this->referenceForce = 2.0f + 1.0f * sin(2.0f * _PI * frequency * t * 0.01f); 
            }
            break;
            
        case ForceType::MAX:
            // Maximum continuous force
            this->referenceForce = MAX_TENDON_FORCE; 
            break;
            
        case ForceType::TENDON_MAX:
            // 50% of maximum continuous force for tendon stretching test
            this->referenceForce = MAX_TENDON_FORCE/2; 
            break;
            
        case ForceType::TENDON_SIN:
            // Sinusoidal force between 10% and 50% of maximum continuous force at 1 Hz
            {
                float tSec = t * 0.01f; // Convert timestep to seconds (assuming 10ms loop)
                this->referenceForce = (0.3f * MAX_TENDON_FORCE) + 
                                      (0.2f * MAX_TENDON_FORCE * sin(2.0f * _PI * 1.0f * tSec));
            }
            break;
    }
}

// Implementation now matches declaration in header
float ForceControl::forcePID(float motor_angle, ForceType forceType)
{
    static float prevErr = 0.0f;
    static float intErr = 0.0f;
    static int timeStep = 0;

    // Generate reference force
    this->forceGeneration(forceType, timeStep++);
    

    //if (this->motorEncoder == nullptr || this->seaEncoder == nullptr) {        
    //    Serial.println("Error: Encoders not initialized");
    //    return 0.0f;
    //}
    // Get force from the encoders
    float SeaForce = encoderToForce(motor_angle, *this->seaEncoder);
    
    // Error
    float error = this->referenceForce - 0;//SeaForce;
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
    pidCurrent = Ff * this->referenceForce + 
                     Kp * error + 
                     Ki * intErr + 
                     Kd * dErr;
    
    // Saturation limits
    if (pidCurrent > MAX_CURRENT) {
        pidCurrent = MAX_CURRENT;
    } else if (pidCurrent < -MAX_CURRENT) {
        pidCurrent = -MAX_CURRENT; 
    }
    
    return pidCurrent;
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