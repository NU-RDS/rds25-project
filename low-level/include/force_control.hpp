#ifndef FORCE_CONTROL_HPP
#define FORCE_CONTROL_HPP

#include <Arduino.h>
#include "encoder.hpp"

// Constants
#define _PI 3.14159265359f
#define MAX_TENDON_FORCE 5.0f
#define ANTI_WINDUP_F 5.0f
#define R_ENCODER_PULLEY 0.02f  // 20mm radius in meters
#define MAX_CURRENT 10 // A

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
    float pidCurrent;  // PID output

    ForceType forceType;
    Encoder* motorEncoder;
    Encoder* seaEncoder;


public:
    // Constructor
    ForceControl(float ff, float kp, float ki, float kd, float ks);
    
    // Convert encoder reading to force
    float encoderToForce(float motor_angle, Encoder& seaEncoder);
    
    // Generate reference force based on type
    void forceGeneration(ForceType forceType, int t);
    
    // PID controller for force - updated to match implementation
    float forcePID(float motor_angle, ForceType forceType);

    // Abstracting SEA encoder angle
    float getSeaEncoderAngle();
    
    // Getters
    float getReferenceForce() { return this->referenceForce; }
    float getPidCurrent() { return this->pidCurrent; }
    float getFf() {return this->Ff; }
    float getKp() {return this->Kp; }
    float getKi() {return this->Ki; }
    float getKd() {return this->Kd; }
    ForceType getForceType() { return this->forceType; }

    // Setters
    void setFf(float ff) {this->Ff = ff;}
    void setKp(float kp) {this->Kp = kp;}
    void setKi(float ki) {this->Ki = ki;}
    void setKd(float kd) {this->Kd = kd;}
    void setForceType(int typeIndex);
    void setMotorEncoder(int encoderCS);
    void setSeaEncoder(int encoderCS);

};

#endif // FORCE_CONTROL_HPP