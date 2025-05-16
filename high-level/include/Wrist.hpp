#ifndef WRIST_HPP
#define WRIST_HPP

#include <memory>
#include <string>
#include <unordered_map>

#include "Joint.hpp"

class Wrist {
    private:
        std::unique_ptr<Joint> Roll;
        std::unique_ptr<Joint> Pitch;
        std::unique_ptr<Joint> Yaw;
    
    public:
        Wrist();
        ~Wrist() = default;

        void kinematics();
        void setWristOrientation(double roll_desired, double pitch_desired, double yaw_desired);
        void sendTorqueCommand(double T_roll, double T_pitch, double T_yaw);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();
};

#endif // WRIST_HPP