#ifndef FORCE_CONTROL_HPP
#define FORCE_CONTROL_HPP

#include <Arduino.h>
#include "encoder.hpp"

// Constants
#define _PI 3.14159265359f
#define MAX_TENDON_FORCE 5.0f
#define ANTI_WINDUP_F 5.0f
#define R_ENCODER_PULLEY 0.02f  // 20mm radius in meters

// Force pattern types
enum class ForceType {
    STEP,
    SIN,
    MAX,
    TENDON_MAX,
    TENDON_SIN
};

class ForceControl {
private:
    float Ff;  // Feedforward gain
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
    float Ks;  // Spring constant
    
    float referenceForce;  // Desired force
    float resultantForce;  // Actual force

public:
    // Constructor
    ForceControl(float ff, float kp, float ki, float kd, float ks);
    
    // Convert encoder reading to force
    float encoderToForce(Encoder encoder);
    
    // Generate reference force based on type
    void forceGeneration(ForceType forceType, int t);
    
    // Null space conversion for resultant force
    float nullSpaceConvertion();
    
    // PID controller for force
    float forcePID(int encoderCS, int encoderId, ForceType forceType);
    
    // Print force values for debugging
    void forcePrint();
    
    // Getters
    float getReferenceForce() { return referenceForce; }
    float getResultantForce() { return resultantForce; }
};

#endif // FORCE_CONTROL_HPP