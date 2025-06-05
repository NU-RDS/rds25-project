#include "TendonKinematics.hpp"

// Convention for order of joint_torques:
// [dex splay, dex mcp, dex pip, dex dip, pow grasp, wrist yaw, wrist pitch]
// Conention for order of motor torques:
// [dex splay, dex pip, dex extensor, dex dip, pow grasp, wrist yaw, wrist pitch]

std::vector<double> TendonKinematics::getMotorTorques(std::vector<double> joint_torques) {
    std::vector<double> motor_torques(7, 0.0);

    // Print joint torques for debugging
    Serial.print("[KINEMATICS] Joint torques: [");
    for (size_t i = 0; i < joint_torques.size(); i++) {
        Serial.print(joint_torques[i], 3);
        if (i < joint_torques.size() - 1) Serial.print(", ");
    }
    Serial.println("]");

    // Dexterous finger torques
    for (size_t i = 0; i < 4; ++i) {
        motor_torques[i] = 1/MOTOR_RADIUS * (J_dex[i][0] * joint_torques[0] + J_dex[i][1] * joint_torques[1] +
                           J_dex[i][2] * joint_torques[2] + J_dex[i][3] * joint_torques[3] + null_dex[i] * 0.1f); // Adding a small null space torque;
    }

    // Power finger torque
    motor_torques[4] = 1/MOTOR_RADIUS * (J_pow * joint_torques[4]);

    // Wrist torques
    motor_torques[5] = 1/R_motor * (R_pow * joint_torques[4] + R_splay * joint_torques[0] + R_mcp * joint_torques[1]
                                    + R_pip * joint_torques[2] + R_extensor * joint_torques[3]) + joint_torques[5];
    motor_torques[6] = 1/R_motor * (R_pow * joint_torques[4] + R_splay * joint_torques[0] + R_mcp * joint_torques[1]
                                    + R_pip * joint_torques[2] + R_extensor * joint_torques[3] + R_yaw * motor_torques[5]) + joint_torques[6];

    // Print calculated motor torques (unmodified)
    Serial.print("[KINEMATICS] Calculated motor torques: [");
    for (size_t i = 0; i < motor_torques.size(); i++) {
        Serial.print(motor_torques[i], 3);
        if (i < motor_torques.size() - 1) Serial.print(", ");
    }
    Serial.println("]");

    return motor_torques;
}

float TendonKinematics::RadToDeg(float ang)
{
    return ang / M_PI * 180.;
}

// degree to radian
float TendonKinematics::DegToRad(float ang)
{
    return ang * M_PI / 180.;
}

// revolutions to degrees
float TendonKinematics::RevToDeg(float encoder)
{
    return encoder * 360.0f;
}

// motor angle to shaft angle
float TendonKinematics::toShaft(float ang)
{
    return ang / GEAR_RATIO;
}

std::vector<float> TendonKinematics::motorToJointAngle(float motor_pitch, float motor_yaw) {
    float joint_pitch = (R_motor / R_pitch) * motor_pitch;
    float joint_yaw = (R_motor * motor_yaw - R_pitch * joint_pitch) / R_yaw;
    std::vector<float> result = {joint_yaw, joint_pitch};

    return result;
}