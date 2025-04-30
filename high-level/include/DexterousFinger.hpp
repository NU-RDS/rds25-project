#include <memory>
#include <unordered_map>

#include "Joint.hpp"

class DexterousFinger {
    private:
        std::unique_ptr<Joint> PIP;
        std::unique_ptr<Joint> DIP;
        std::unique_ptr<Joint> MCP;
        std::unique_ptr<Joint> Splain;

    public:
        DexterousFinger();
        ~DexterousFinger() = default;
        
        void kinematics();
        void setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splain_desired);
        void sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splain);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();
};