#ifndef TENDON_KINEMATICS_HPP
#define TENDON_KINEMATICS_HPP

#include <vector>
#include <math.h>

#include <Arduino.h>
#include "RDS_constants.hpp"

class TendonKinematics{
    private:
        std::vector<std::vector<double>> J_dex;
        std::vector<double> null_dex;
        float J_pow;
        float R_pitch = 24.0f;
        float R_yaw = 24.0f;
        float R_pow = 13.0f;
        float R_mcp = 7.5f;
        float R_splay = 7.5f;
        float R_extensor = 13.0f;
        float R_pip = 18.5f;
        float R_motor = 5.0f;
        const float GEAR_RATIO = 36.0f;

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

        std::vector<double> getMotorTorques(std::vector<double> joint_torques);

        float RadToDeg(float ang);
        float DegToRad(float ang);
        float RevToDeg(float encoder);
        float RevToRad(float encoder);
        float toShaft(float ang);

        std::vector<float> motorToJointAngleWrist(float pitch, float yaw);
        std::vector<float> motorToJointAngleDex(float splay, float mcp,  float pip, float dip);
                        
};

#endif // TENDON_KINEMATICS_HPP