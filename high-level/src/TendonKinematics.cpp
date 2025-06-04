#include "TendonKinematics.hpp"

// Convention for order of joint_torques:
// [dex splay, dex mcp, dex pip, dex dip, pow grasp, wrist yaw, wrist pitch]
// Conention for order of motor torques:
// [dex splay, dex pip, dex extensor, dex dip, pow grasp, wrist yaw, wrist pitch]

std::array<float, 7> TendonKinematics::getMotorTorques(std::array<float, 7> joint_torques) {
    std::array<float, 7> motor_torques = {0.0f};

    // Dexterous finger torques
    for (size_t i = 0; i < 4; ++i) {
        motor_torques[i] = J_dex[i][0] * joint_torques[0] + J_dex[i][1] * joint_torques[1] +
                           J_dex[i][2] * joint_torques[2] + J_dex[i][3] * joint_torques[3] + null_dex[i] * 0.1f; // Adding a small null space torque;
    }

    // Power finger torque
    motor_torques[4] = J_pow * joint_torques[4];

    // Wrist torques
    motor_torques[5] = 1/R_motor * (R_pow * motor_torques[4] + R_splay * motor_torques[0] + R_mcp * motor_torques[1]
                                    + R_pip * motor_torques[2] + R_extensor * motor_torques[3]) + joint_torques[5];
    motor_torques[6] = 1/R_motor * (R_pow * motor_torques[4] + R_splay * motor_torques[0] + R_mcp * motor_torques[1]
                                    + R_pip * motor_torques[2] + R_extensor * motor_torques[3] + R_yaw * motor_torques[5]) + joint_torques[6];

    return motor_torques;
}