#include "PositionControl.hpp"

PositionControl::PositionControl(float ff, float kp, float ki, float kd, float ks) :
    Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks), 
    referencePosition(0.0f), resultantPosition(0.0f) {}

float PositionControl::getJointAngle(Encoder& encoder)
{
    return encoder.readEncoderDeg();
}

// NOT IMPLEMENTED 
void PositionControl::positionGeneration(PositionType positionType, int t) {
    switch (positionType) {
        case PositionType::STEP:
            break;
            
        case PositionType::SIN:
            break;
        
    }
}

float PositionControl::forcePID(int encoderCS, int encoderId, PositionControl::PositionType forceType)
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
    this->positionGeneration(forceType, timeStep++);
    
    // Current force from the encoder
    Encoder encoder(encoderCS, encoderId);
    float currentPosition = getJointAngle(encoder);
    
    // Error
    float error = this->referencePosition - currentPosition;
    intErr += error;
    
    // Anti-windup
    if (intErr > ANTI_WINDUP_P) {
        intErr = ANTI_WINDUP_P;
    } else if (intErr < -ANTI_WINDUP_P) {
        intErr = -ANTI_WINDUP_P;
    }
    
    // Derivative
    float dErr = error - prevErr;
    prevErr = error;
    
    // PID
    resultantPosition = Ff * this->referencePosition + Kp * error + Ki * intErr + Kd * dErr;
    
    // Saturation limits
    if (resultantPosition > ROM_MAX) {
        resultantPosition = ROM_MAX;
    } else if (resultantPosition < 0.0f) {
        resultantPosition = 0.0f; 
    }
    
    // Apply null space conversion (probabaly not needed)
    float motorTorque = this->nullspaceConversion();
    return motorTorque;
}

void PositionControl::positionPrint()
{
    Serial.print(referencePosition);
    Serial.print(",");
    Serial.println(resultantPosition);
    delay(10);
}