#ifndef POWER_FINGER_HPP
#define POWER_FINGER_HPP

#include <Arduino.h>
#include <memory>
#include <string>
#include <unordered_map>

#include "Joint.hpp"
#include "RDS_constants.hpp"

class PowerFinger {
    private:
        std::unique_ptr<Joint> Grasp;

        double J_pow = 1.0f;

    public:
        PowerFinger();
        ~PowerFinger() = default;
        
        void kinematics();

        void setJointAngles(double grasp_desired);
        void sendTorqueCommand(double T_grasp);
        Joint* getGrasp() { return Grasp.get(); }

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();
        double calculateControl();
};

#endif // POWER_FINGER_HPP