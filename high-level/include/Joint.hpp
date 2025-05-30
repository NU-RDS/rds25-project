#ifndef JOINT_HPP
#define JOINT_HPP

#include <Arduino.h>
#include <string>
#include <cmath>
#include <memory>

#include "PositionControl.hpp"

class Joint {
    protected:
        std::string name;
        double currentPosition; // radians
        double desiredPosition; // radians
        double commandTorque;   // Nm to be sent to MCU
        double controlSignal;
        double encoderValue;

        // Joint limits
        double maxLimit;
        double minLimit;

        // Controller for joint
        std::unique_ptr<PositionControl> controller;
    
    public:
        Joint(const std::string& name);
        ~Joint() = default;
    
        // Getters and setters
        const std::string& getName();
        double getCurrentPosition();
        double getDesiredPosition();
        double getCommandTorque();
        double getControlSignal();
    
        void setCurrentPosition(double current_pos);
        void setDesiredPosition(double desired_pos);
        void setCommandTorque(double command_torque);
        
        // Control functionality
        double calculateControlSignal();
};

#endif // JOINT_HPP