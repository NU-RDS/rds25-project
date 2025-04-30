#include <string>
#include <cmath>

class Joint {
    protected:
        std::string name;
        double currentPosition; // radians
        double desiredPosition; // radians
        double currentVelocity; // rad/s
        double currentTorque;   // Nm
        double commandTorque;   // Nm to be sent to MCU

        // Joint limits
        double maxLimit;
        double minLimit;
    
    public:
        Joint(const std::string& name);
        ~Joint();
    
        // Getters and setters
        const std::string& getName();
        double getCurrentPosition();
        double getDesiredPosition();
        double getCurrentVelocity();
        double getCurrentTorque();
        double getCommandTorque();
    
        void setCurrentPosition(double current_pos);
        void setDesiredPosition(double desired_pos);
        void setCurrentVelocity(double current_vel);
        void setCurrentTorque(double current_torque);
        void setCommandTorque(double command_torque);
    };