#include "TendonKinematics.hpp"

// Convention for order of joint_torques:
// [dex splay, dex mcp, dex pip, dex dip, pow grasp, wrist yaw, wrist pitch]
// Conention for order of motor torques:
// [dex splay, dex pip, dex extensor, dex dip, pow grasp, wrist yaw, wrist pitch]

/**
 * @brief Computes the required motor torques based on the provided joint torques.
 *
 * This function calculates the torques that need to be applied by each motor to achieve the desired joint torques
 * for a tendon-driven robotic hand. The calculation takes into account the kinematic relationships between joints
 * and motors, including the dexterous fingers, power finger, and wrist joints. It also applies a small null space
 * torque to the dexterous fingers for stability.
 *
 * @param joint_torques A vector of joint torques (size 7), where each element corresponds to a specific joint.
 * @return std::vector<double> A vector of calculated motor torques (size 7), where each element corresponds to a specific motor.
 */
std::vector<double> TendonKinematics::getMotorTorques(std::vector<double> joint_torques) {
    std::vector<double> motor_torques(7, 0.0);

    // Print joint torques for debugging
    // Serial.print("[KINEMATICS] Joint torques: [");
    // for (size_t i = 0; i < joint_torques.size(); i++) {
    //     Serial.print(joint_torques[i], 3);
    //     if (i < joint_torques.size() - 1) Serial.print(", ");
    // }
    // Serial.println("]");

    // Dexterous finger torques
    for (size_t i = 0; i < 4; ++i) {
        motor_torques[i] = 1/MOTOR_RADIUS * (J_dex[i][0] * joint_torques[0] + J_dex[i][1] * joint_torques[1] +
                           J_dex[i][2] * joint_torques[2] + J_dex[i][3] * joint_torques[3] + null_dex[i] * 0.1f); // Adding a small null space torque;
    }

    // Power finger torque
    motor_torques[4] = 1/MOTOR_RADIUS * (J_pow * joint_torques[4]);

    // Wrist torques
    motor_torques[5] = 1/R_motor * (R_pow * motor_torques[4] + R_splay * motor_torques[0] + R_mcp * motor_torques[1]
                                    + R_pip * motor_torques[2] + R_extensor * motor_torques[3]) + joint_torques[5];
    motor_torques[6] = 1/R_motor * (R_pow * motor_torques[4] + R_splay * motor_torques[0] + R_mcp * motor_torques[1]
                                    + R_pip * motor_torques[2] + R_extensor * motor_torques[3] + R_yaw * motor_torques[5]) + joint_torques[6];

    // Print calculated motor torques (unmodified)
    // Serial.print("[KINEMATICS] Calculated motor torques: [");
    // for (size_t i = 0; i < motor_torques.size(); i++) {
    //     Serial.print(motor_torques[i], 3);
    //     if (i < motor_torques.size() - 1) Serial.print(", ");
    // }
    // Serial.println("]");

    return motor_torques;
}

/**
 * @brief Converts an angle from radians to degrees.
 *
 * @param ang Angle in radians.
 * @return Angle converted to degrees.
 */
float TendonKinematics::RadToDeg(float ang)
{
    return ang / M_PI * 180.;
}

// degree to radian
/**
 * @brief Converts an angle from degrees to radians.
 *
 * @param ang Angle in degrees.
 * @return Angle in radians.
 */
float TendonKinematics::DegToRad(float ang)
{
    return ang * M_PI / 180.;
}

// revolutions to degrees
/**
 * @brief Converts a value in revolutions to degrees.
 *
 * This function takes an encoder value representing revolutions and converts it to degrees.
 *
 * @param encoder The value in revolutions to be converted.
 * @return The equivalent value in degrees.
 */
float TendonKinematics::RevToDeg(float encoder)
{
    return encoder * 360.0f;
}

/**
 * @brief Converts encoder revolutions to radians.
 *
 * This function takes the number of revolutions measured by the encoder
 * and converts it to the equivalent angle in radians.
 *
 * @param encoder The number of revolutions from the encoder.
 * @return The angle in radians.
 */
float TendonKinematics::RevToRad(float encoder)
{
    return encoder * 2 * M_PI;
}

// motor angle to shaft angle
/**
 * @brief Converts an angular value from the tendon side to the shaft side.
 *
 * This function takes an angle (in radians or degrees, depending on system convention)
 * measured at the tendon and converts it to the equivalent angle at the shaft,
 * accounting for the gear ratio between the tendon and the shaft.
 *
 * @param ang The angular value at the tendon.
 * @return The corresponding angular value at the shaft.
 */
float TendonKinematics::toShaft(float ang)
{
    return ang / GEAR_RATIO;
}

/**
 * @brief Converts motor angles to corresponding joint angles.
 *
 * This function computes the joint pitch and yaw angles based on the provided
 * motor pitch and yaw angles, using the tendon kinematics model parameters.
 *
 * @param motor_pitch The pitch angle of the motor (in radians or degrees, depending on convention).
 * @param motor_yaw The yaw angle of the motor (in radians or degrees, depending on convention).
 * @return std::vector<float> A vector containing the computed joint yaw and joint pitch angles,
 *         in that order: {joint_yaw, joint_pitch}.
 */
std::vector<float> TendonKinematics::motorToJointAngle(float motor_pitch, float motor_yaw) {
    float joint_pitch = (R_motor / R_pitch) * motor_pitch;
    float joint_yaw = (R_motor * motor_yaw - R_pitch * joint_pitch) / R_yaw;
    std::vector<float> result = {joint_yaw, joint_pitch};

    return result;
}