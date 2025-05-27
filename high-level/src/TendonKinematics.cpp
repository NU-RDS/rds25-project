#include "TendonKinematics.hpp"

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
    for (size_t i = 0; i < 7; ++i) {
        motor_torques[5 + i] = J_wrist[i][0] * joint_torques[5] + J_wrist[i][1] * joint_torques[6];
    }

    return motor_torques;
}