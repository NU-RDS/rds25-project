#ifndef DEXTEROUS_FINGER_HPP
#define DEXTEROUS_FINGER_HPP

#include <Arduino.h>
#include <memory>
#include <unordered_map>

#include "Joint.hpp"
#include "RDS_constants.hpp"

/**
 * @class DexterousFinger
 * @brief Represents a dexterous robotic finger with four joints: PIP, DIP, MCP, and Splay.
 *
 * This class encapsulates the kinematics and control of a dexterous finger, providing interfaces
 * for setting joint positions, sending torque commands, and retrieving joint angles.
 * It manages the following joints using unique pointers:
 *   - PIP (Proximal Interphalangeal)
 *   - DIP (Distal Interphalangeal)
 *   - MCP (Metacarpophalangeal)
 *   - Splay (Lateral movement)
 *
 * The class also stores constant matrices for dexterous control and null space configuration.
 *
 * Public Methods:
 *   - DexterousFinger(): Constructor.
 *   - ~DexterousFinger(): Destructor (default).
 *   - Joint* getPIP(): Returns a pointer to the PIP joint.
 *   - Joint* getDIP(): Returns a pointer to the DIP joint.
 *   - Joint* getMCP(): Returns a pointer to the MCP joint.
 *   - Joint* getSplay(): Returns a pointer to the Splay joint.
 *   - void kinematics(): Computes or updates the kinematics of the finger.
 *   - void setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splay_desired):
 *       Sets the desired positions for each joint.
 *   - void sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splay):
 *       Sends torque commands to each joint.
 *   - const std::unordered_map<std::string, double> getDesiredJointAngles():
 *       Retrieves the desired angles for all joints.
 *   - const std::unordered_map<std::string, double> getCurrentJointAngles():
 *       Retrieves the current angles for all joints.
 *   - std::vector<double> calculateControl():
 *       Calculates and returns the control signals for the finger.
 */
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
        Joint* getSplay() { return Splay.get(); }

        void kinematics();
        
        void setDexterousFingerPositions(double pip_desired, double dip_desired, double mcp_desired, double splay_desired);
        void sendTorqueCommand(double T_pip, double T_dip, double T_mcp, double T_splay);

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();

        std::vector<double> calculateControl();
};

#endif // DEXTEROUS_FINGER_HPP