#ifndef TENDON_KINEMATICS_HPP
#define TENDON_KINEMATICS_HPP

#include <array>
#include <math.h>

class TendonKinematics{
    private:
        std::array<std::array<float, 4>, 4> J_dex;
        std::array<float, 4> null_dex;
        float J_pow;
        std::array<std::array<float, 2>, 7> J_wrist;

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
            J_pow = 1.0;
            J_wrist = {{
                {1.0, 1.0},
                {1.0, -1.0},
                {-1.0, 1.0},
                {-1.0, -1.0},
                {1.5, 1.5},
                {-1.5, -1.5},
                {2.5, -2.5}
            }};
        }

        std::array<float, 7> getMotorTorques(std::array<float, 7> joint_torques);
                        
};

#endif