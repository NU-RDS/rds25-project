#ifndef TENDON_KINEMATICS_HPP
#define TENDON_KINEMATICS_HPP

#include <array>
#include <math.h>

class TendonKinematics{
    private:
        std::array<std::array<float, 4>, 4> J_dex;
        std::array<float, 4> null_dex;
        float J_pow;
        float R_pitch = 24.0f;
        float R_yaw = 24.0f;
        float R_pow = 13.0f;
        float R_mcp = 7.5f;
        float R_splay = 7.5f;
        float R_extensor = 13.0f;
        float R_pip = 18.5f;
        float R_motor = 5.0f;

    public:
        TendonKinematics() {
            // Initialize Jacobian matrices
            J_dex = {{
                {1.0f, -2.0f, -1.2f, -1.2f},
                {-1.0f, 0.0f, 0.0f, 0.0f},
                {-1.0f, 2.0f, 1.2f, 1.2f},
                {1.0f, 2.52f, 1.2f, 1.2f}
            }};
            null_dex = {0.0f, 0.0f, -1.0f, 1.0f};
            J_pow = 1.0; //have to change 
        }

        std::array<float, 7> getMotorTorques(std::array<float, 7> joint_torques);
                        
};

#endif