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
            {1.0, -2.0, -1.2, -1.2},
            {-1.0, 0.0, 0.0, 0.0},
            {-1.0, 2.0, 1.2, 1.2},
            {1.0, 2.52, 1.2, 1.2}
        };
        const std::vector<double> null_dex = {0.0, 0.0, -1.0, 1.0};


    public:
        DexterousFinger();
        ~DexterousFinger() = default;

        Joint* getPIP() { return PIP.get(); }
        Joint* getDIP() { return DIP.get(); }
        Joint* getMCP() { return MCP.get(); }
        Joint* getSplain() { return Splay.get(); }

        void kinematics();
        
        void setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splay_desired);
        void sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splain);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();

        std::vector<double> calculateControl();
};

#endif // DEXTEROUS_FINGER_HPP