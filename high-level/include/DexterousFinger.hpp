#ifndef DEXTEROUS_FINGER_HPP
#define DEXTEROUS_FINGER_HPP

#include <Arduino.h>
#include <memory>
#include <unordered_map>

#include "Joint.hpp"
#include "RDS_constants.hpp"

class DexterousFinger {
    private:
        std::unique_ptr<Joint> PIP;
        std::unique_ptr<Joint> DIP;
        std::unique_ptr<Joint> MCP;
        std::unique_ptr<Joint> Splay;

        const std::vector<std::vector<double>> J_dex = {
            {-0.137, 0.525, -2.16},
            {-0.472, 0.328, -1.283},
            {-0.401, 0.361, -1.101},
            {0.264, 0.164, -0.225}
        };
        const std::vector<double> null_dex = {-0.519, 2.0, -1.518, 1.0};


    public:
        DexterousFinger();
        ~DexterousFinger() = default;

        Joint* getPIP() { return PIP.get(); }
        Joint* getDIP() { return DIP.get(); }
        Joint* getMCP() { return MCP.get(); }
        Joint* getSplay() { return Splay.get(); }

        void kinematics();
        
        void setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splay_desired);
        void sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splay);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();

        std::vector<double> calculateControl();
};

#endif // DEXTEROUS_FINGER_HPP