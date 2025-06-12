#ifndef WRIST_HPP
#define WRIST_HPP

#include <Arduino.h>
#include <memory>
#include <string>
#include <unordered_map>

#include "Joint.hpp"
#include "RDS_constants.hpp"

class Wrist {
    private:
        std::unique_ptr<Joint> Pitch;
        std::unique_ptr<Joint> Yaw;
    
    public:
        Wrist();
        ~Wrist() = default;

        Joint* getPitch() { return Pitch.get(); }
        Joint* getYaw() { return Yaw.get(); }

        void kinematics();
        void setWristOrientation(double pitch_desired, double yaw_desired);
        void sendTorqueCommand(double T_pitch, double T_yaw);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();

        std::vector<double> calculateControl();
};

#endif // WRIST_HPP