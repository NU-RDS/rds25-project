#ifndef JOINT_HPP
#define JOINT_HPP

#include <Arduino.h>
#include <string>
#include <cmath>
#include <memory>

#include "PositionControl.hpp"

/**
 * @class Joint
 * @brief Represents a robotic joint with position control capabilities.
 *
 * The Joint class encapsulates the state and control logic for a single robotic joint,
 * including its name, position, torque command, and associated controller. It provides
 * methods to get and set joint parameters, as well as to calculate the control signal
 * required to achieve a desired position.
 *
 * @note This class assumes the use of a PositionControl controller for closed-loop control.
 *
 * @param std::string name
 *      The name identifier for the joint.
 * @param double currentPosition
 *      The current position of the joint in radians.
 * @param double desiredPosition
 *      The target position for the joint in radians.
 * @param double commandTorque
 *      The torque command (in Nm) to be sent to the motor control unit (MCU).
 * @param double controlSignal
 *      The computed control signal for the joint.
 * @param double encoderValue
 *      The raw value from the joint's encoder.
 * @param double motorValue
 *      The value representing the motor's state.
 * @param double encoderOffset
 *      The offset applied to the encoder value for calibration.
 * @param double maxLimit
 *      The maximum allowable position for the joint.
 * @param double minLimit
 *      The minimum allowable position for the joint.
 * @param std::unique_ptr<PositionControl> controller
 *      The position controller used for closed-loop control of the joint.
 *
 */
class Joint {
    protected:
        std::string name;
        double currentPosition; // radians
        double desiredPosition; // radians
        double commandTorque;   // Nm to be sent to MCU
        double controlSignal;
        double encoderValue;
        double motorValue;
        double encoderOffset;

        // Joint limits
        double maxLimit;
        double minLimit;

        // Controller for joint
        std::unique_ptr<PositionControl> controller;
    
    public:
        Joint(const std::string& name, const double kp, const double ki, const double kd);
        ~Joint() = default;
    
        // Getters and setters
        const std::string& getName();
        double getCurrentPosition();
        double getDesiredPosition();
        double getCommandTorque();
        double getControlSignal();
        double getMotorValue();
        double getEncoderOffset();
    
        void setCurrentPosition(double current_pos);
        void setDesiredPosition(double desired_pos);
        void setCommandTorque(double command_torque);
        void setControlSignal(double control_signal);
        void setMotorValue(double motor_value);
        void setEncoderOffset(double encoder_offset);
        
        // Control functionality
        double calculateControlSignal();
};

#endif // JOINT_HPP