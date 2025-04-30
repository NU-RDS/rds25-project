#include <memory>
#include <string>
#include <unordered_map>

#include "Joint.hpp"

class PowerFinger {
    private:
        std::unique_ptr<Joint> Grasp;

    public:
        PowerFinger();
        ~PowerFinger();
        
        void kinematics();
        void setJointAngles(double grasp_desired);
        void sendTorqueCommand(double T_grasp);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();
};