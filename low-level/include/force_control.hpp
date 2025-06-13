/**
 * @file force_control.hpp
 * @brief Defines the ForceControl class for force-based control using PID and feedforward strategies.
 *
 * This header provides the ForceControl class, which implements force control logic for a system
 * using a Series Elastic Actuator (SEA) and an encoder. The class supports various force generation
 * patterns (step, sinusoidal, max, tendon-specific) and provides methods for converting encoder
 * readings to force, generating reference forces, and running a PID controller to compute the
 * required current for force tracking.
 *
 * Constants:
 *   - _PI: Value of pi.
 *   - MAX_TENDON_FORCE: Maximum allowable tendon force (N).
 *   - ANTI_WINDUP_F: Anti-windup factor for PID integrator.
 *   - R_ENCODER_PULLEY: Radius of the encoder pulley (meters).
 *   - MAX_CURRENT: Maximum allowable current (A).
 *
 * Enumerations:
 *   - ForceType: Types of force patterns supported (STEP, SIN, MAX, TENDON_MAX, TENDON_SIN).
 *
 * Class ForceControl:
 *   - Implements force control using PID and feedforward.
 *   - Provides methods for:
 *       - Converting encoder readings to force.
 *       - Generating reference force patterns.
 *       - Running a PID controller for force tracking.
 *       - Getting and setting control parameters and force types.
 *       - Associating an encoder for SEA angle measurement.
 *
 * Usage:
 *   - Instantiate with desired PID and feedforward gains.
 *   - Set the force pattern and encoder as needed.
 *   - Use forcePID() to compute control output based on current state.
 */
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
    Encoder* seaEncoder;


public:
    // Constructor
    ForceControl(float ff, float kp, float ki, float kd, float ks);
    
    // Convert encoder reading to force
    float encoderToForce(float motor_angle, float sea_angle);
    
    // Generate reference force based on type
    void forceGeneration(ForceType forceType, int t);
    
    // PID controller for force - updated to match implementation
    float forcePID(float motor_angle, float sea_angle, ForceType forceType);

    // Getters
    float getReferenceForce() { return this->referenceForce; }
    float getPidCurrent() { return this->pidCurrent; }
    float getFf() {return this->Ff; }
    float getKp() {return this->Kp; }
    float getKi() {return this->Ki; }
    float getKd() {return this->Kd; }
    ForceType getForceType() { return this->forceType; }
    float getSeaEncoderAngle();


    // Setters
    void setFf(float ff) {this->Ff = ff;}
    void setKp(float kp) {this->Kp = kp;}
    void setKi(float ki) {this->Ki = ki;}
    void setKd(float kd) {this->Kd = kd;}
    void setForceType(int typeIndex);
    void setSeaEncoder(int encoderCS);

};

#endif // FORCE_CONTROL_HPP