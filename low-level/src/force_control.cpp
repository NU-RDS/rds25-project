#include "force_control.hpp"

ForceControl::ForceControl(float ff, float kp, float ki, float kd, float ks) :
    Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks), 
    referenceForce(0.0f), resultantForce(0.0f) {}

float ForceControl::encoderToForce(Encoder encoder)
{
    float angle = encoder.readEncoderDeg();
    float distance = R_ENCODER_PULLEY * (angle / 180 * _PI);
    return distance * this->Ks;
}

void ForceControl::forceGeneration(ForceType forceType, int t) {
    switch (forceType) {
        case ForceType::STEP:
            // Step input
            this->referenceForce = (t % 2 == 0) ? 1.0f : 3.0f;
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

// SUBJECT TO CHANGE
float ForceControl::nullSpaceConvertion()
{
    if (resultantForce < 0)
    {
        return 0;
    }
    return resultantForce;
}

float ForceControl::forcePID(int encoderCS, int encoderId, ForceType forceType)
{
    // Use static maps to track PID state per encoder
    static std::map<int, float> prevErrMap;
    static std::map<int, float> intErrMap;
    static std::map<int, int> timeStepMap;
    
    // Initialize if not already present
    if (prevErrMap.find(encoderId) == prevErrMap.end()) {
        prevErrMap[encoderId] = 0.0f;
        intErrMap[encoderId] = 0.0f;
        timeStepMap[encoderId] = 0;
    }
    
    // Get references to the PID state for this encoder
    float& prevErr = prevErrMap[encoderId];
    float& intErr = intErrMap[encoderId];
    int& timeStep = timeStepMap[encoderId];

    // Generate reference force
    this->forceGeneration(forceType, timeStep++);
    
    // Current force from the encoder
    Encoder encoder(encoderCS, encoderId);
    float currentForce = encoderToForce(encoder);
    
    // Error
    float error = this->referenceForce - currentForce;
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
    resultantForce = Ff * this->referenceForce + 
                     Kp * error + 
                     Ki * intErr + 
                     Kd * dErr;
    
    // Saturation limits
    if (resultantForce > MAX_TENDON_FORCE) {
        resultantForce = MAX_TENDON_FORCE;
    } else if (resultantForce < 0.0f) {
        resultantForce = 0.0f; 
    }
    
    // Apply null space conversion
    float motorTorque = this->nullSpaceConvertion();
    return resultantForce;
}

void ForceControl::forcePrint()
{
    Serial.print(referenceForce);
    Serial.print(",");
    Serial.println(resultantForce);
    delay(10);
}