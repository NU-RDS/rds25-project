#ifndef TENDON_KINEMATICS_HPP
#define TENDON_KINEMATICS_HPP

#include <vector>
#include <math.h>

#include <Arduino.h>
#include "RDS_constants.hpp"

/**
 * @class TendonKinematics
 * @brief Provides kinematic calculations for tendon-driven mechanisms, including Jacobian-based transformations and utility functions for angle conversions.
 *
 * This class encapsulates the kinematic relationships for a tendon-driven system, such as a robotic finger or hand.
 * It maintains internal representations of Jacobian matrices and tendon routing radii, and provides methods to convert
 * between joint torques and motor torques, as well as between various angle representations (radians, degrees, revolutions).
 *
 * Private Members:
 * - J_dex: Jacobian matrix for dexterous tendon mapping.
 * - null_dex: Null space vector for dexterous tendon mapping.
 * - J_pow: Scalar for power tendon mapping (to be updated as needed).
 * - R_pitch, R_yaw, R_pow, R_mcp, R_splay, R_extensor, R_pip, R_motor: Radii for different tendon routing paths (in mm).
 * - GEAR_RATIO: Constant gear ratio between motor and output.
 *
 * Public Methods:
 * - TendonKinematics(): Constructor initializing Jacobian matrices and parameters.
 * - std::vector<double> getMotorTorques(std::vector<double> joint_torques): Computes motor torques from given joint torques.
 * - float RadToDeg(float ang): Converts radians to degrees.
 * - float DegToRad(float ang): Converts degrees to radians.
 * - float RevToDeg(float encoder): Converts encoder revolutions to degrees.
 * - float RevToRad(float encoder): Converts encoder revolutions to radians.
 * - float toShaft(float ang): Converts an angle to the corresponding shaft value.
 * - std::vector<float> motorToJointAngle(float pitch, float yaw): Computes joint angles from given motor angles.
 */
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

        std::vector<float> motorToJointAngle(float pitch, float yaw);
                        
};

#endif // TENDON_KINEMATICS_HPP