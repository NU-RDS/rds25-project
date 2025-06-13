#include "force_control.hpp"

/**
 * @brief Constructs a ForceControl object with specified control parameters.
 * 
 * Initializes the feedforward gain, PID gains, static friction compensation, and sets
 * default values for reference force, current PID output, force type, and encoder pointer.
 * 
 * @param ff Feedforward gain for force control.
 * @param kp Proportional gain for the PID controller.
 * @param ki Integral gain for the PID controller.
 * @param kd Derivative gain for the PID controller.
 * @param ks Static friction compensation gain.
 */
ForceControl::ForceControl(float ff, float kp, float ki, float kd, float ks) :
    Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks), 
    referenceForce(0.0f), pidCurrent(0.0f), 
    forceType(ForceType::STEP), seaEncoder(nullptr) {  // Initialize encoder pointer to nullptr
}

/**
 * @brief Converts the difference between motor and SEA (Series Elastic Actuator) encoder angles to an estimated force.
 *
 * This function calculates the force based on the difference between the motor angle and the SEA angle,
 * using a linear and quadratic relationship. The coefficients are likely determined by system calibration.
 *
 * @param motor_angle The current angle of the motor encoder (in radians or degrees, depending on system convention).
 * @param sea_angle The current angle of the SEA encoder (in the same units as motor_angle).
 * @return Estimated force corresponding to the encoder angle difference.
 */
float ForceControl::encoderToForce(float motor_angle, float sea_angle)  // Changed parameter to reference
{
    float diff = motor_angle - sea_angle;
    float force = diff * 0.00522f - 0.0000274 * diff * diff;

    return force;
}

/**
 * @brief Generates a reference force based on the specified force type and time step.
 *
 * This function sets the `referenceForce` member variable according to the selected
 * force generation mode. The force profile can be a step, sinusoidal, maximum, or
 * tendon-specific force, depending on the application requirements.
 *
 * @param forceType The type of force profile to generate. Supported types:
 *   - ForceType::STEP: Generates a step force pattern alternating between 1.0N, 0.0N, 3.0N, and 0.0N every 250 timesteps.
 *   - ForceType::SIN: Generates a sinusoidal force of the form 2 + 1*sin(2πft) N, with f = 1 Hz.
 *   - ForceType::MAX: Sets the force to the maximum continuous tendon force.
 *   - ForceType::TENDON_MAX: Sets the force to 50% of the maximum continuous tendon force.
 *   - ForceType::TENDON_SIN: Generates a sinusoidal force between 10% and 50% of the maximum continuous tendon force at 1 Hz.
 * @param t The current time step (integer), typically incremented every control loop iteration.
 *
 * @note Assumes a control loop period of 10 ms (i.e., t*0.01f gives time in seconds).
 */
void ForceControl::forceGeneration(ForceType forceType, int t) {
    switch (forceType) {
        case ForceType::STEP:
            // Step input
            // this->referenceForce = ((t/500) % 2 == 0) ? 1.0f : 3.0f;
            this->referenceForce = ((t/250) % 4 == 0) ? 1.0f : 
                       ((t/250) % 4 == 1) ? 0.0f : 
                       ((t/250) % 4 == 2) ? 3.0f : 0.0f;
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
/**
 * @brief Computes the PID control current for force control using SEA (Series Elastic Actuator) feedback.
 *
 * This function calculates the control current required to achieve a desired force output
 * by comparing the reference force (generated internally) with the measured force from the encoders.
 * It implements a PID controller with anti-windup and output saturation.
 *
 * @param motor_angle The current angle of the motor encoder (in radians or degrees, depending on implementation).
 * @param sea_angle The current angle of the SEA encoder (in radians or degrees, depending on implementation).
 * @param forceType The type of force profile to generate (used by forceGeneration).
 * @return The computed control current (float) to be applied to the actuator.
 */
float ForceControl::forcePID(float motor_angle, float sea_angle, ForceType forceType)
{
    static float prevErr = 0.0f;
    static float intErr = 0.0f;
    static int timeStep = 0;

    // Generate reference force
    this->forceGeneration(forceType, timeStep++);
    
    // Get force from the encoders
    float SeaForce = encoderToForce(motor_angle, sea_angle);
    
    // Error
    float error = this->referenceForce - SeaForce;
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

/**
 * @brief Sets the type of force to be applied based on the provided index.
 *
 * @param typeIndex An integer representing the desired force type:
 *                  - 0: STEP force
 *                  - 1: SIN force
 *                  - Any other value defaults to STEP force
 */
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

/**
 * @brief Retrieves the current angle from the SEA (Series Elastic Actuator) encoder in degrees.
 *
 * This function checks if the SEA encoder is initialized. If not, it prints an error message
 * and returns 0.0f. Otherwise, it reads and returns the encoder angle in degrees.
 *
 * @return The current SEA encoder angle in degrees as a float. Returns 0.0f if the encoder is not initialized.
 */
float ForceControl::getSeaEncoderAngle(){
    if (this->seaEncoder == nullptr) {
        Serial.println("Error: Sea encoder not initialized");
        return 0.0f;
    }
    return this->seaEncoder->readEncoderDeg();
}

/**
 * @brief Sets the SEA (Series Elastic Actuator) encoder for the ForceControl object.
 *
 * This method deletes any existing encoder instance associated with the object,
 * and creates a new Encoder using the provided chip select (CS) pin.
 *
 * @param encoderCS The chip select pin number for the new SEA encoder.
 */
void ForceControl::setSeaEncoder(int encoderCS)
{
    if (this->seaEncoder != nullptr) {
        delete this->seaEncoder;  
    }
    this->seaEncoder = new Encoder(encoderCS);
}