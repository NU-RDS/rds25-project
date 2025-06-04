#include "SEALookup.hpp"

// Relating tendon force to spring deflection, constants are N/deg
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