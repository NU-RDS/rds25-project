#include "SEALookup.hpp"

// Relating tendon force to spring deflection, constants are N/deg
/**
 * @brief Calculates the torque for a specified joint based on the given angle difference.
 *
 * This function selects the appropriate torque calculation method for the specified joint index.
 * The angle difference (`diff`) should be provided in degrees.
 *
 * @param diff The difference in angle (in degrees) for which the torque is to be calculated.
 * @param jointIndex The index of the joint (0-6) for which the torque is to be calculated.
 *                   Each index corresponds to a specific joint:
 *                   0 - Joint 1
 *                   1 - Joint 2
 *                   2 - Joint 3
 *                   3 - Joint 4
 *                   4 - Joint 5
 *                   5 - Joint 6
 *                   6 - Joint 7
 * @return The calculated torque for the specified joint. Returns 0.0f if the joint index is invalid.
 */
float SEALookup::calculateTorque(float diff, int jointIndex) {
    // diff needs to be in degrees
    switch (jointIndex) {
        case 0: return returnTorque1(diff);
        case 1: return returnTorque2(diff);
        case 2: return returnTorque3(diff);
        case 3: return returnTorque4(diff);
        case 4: return returnTorque5(diff);
        case 5: return returnTorque6(diff);
        case 6: return returnTorque7(diff);
        default: return 0.0f; // Invalid joint index
    }
}

float SEALookup::returnTorque1(float diff) {
    // SEA 2
    return diff * 0.206f; 
}

float SEALookup::returnTorque2(float diff) {
    // SEA 3
    return diff * 0.972f - 0.00432f * diff * diff; 
}

float SEALookup::returnTorque3(float diff) {
    // SEA 4
    return diff * 0.829f - 0.00714f * diff * diff; ; 
}

float SEALookup::returnTorque4(float diff) {
    // SEA 5
    return diff * 0.561f; 
}

float SEALookup::returnTorque5(float diff) {
    // SEA 6
    return diff * 0.45f; 
}

float SEALookup::returnTorque6(float diff) {
    // SEA Wrist
    return diff * 0.104f - 0.00547 * diff * diff;
}

float SEALookup::returnTorque7(float diff) {
    // SEA 8
    return diff * 0.933f - 0.00241f * diff * diff; 
}