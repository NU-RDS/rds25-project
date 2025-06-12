#ifndef SEA_LOOKUP_HPP
#define SEA_LOOKUP_HPP

#include <array>

class SEALookup {
    public:
        SEALookup() = default;
        float returnTorque1(float diff);
        float returnTorque2(float diff);
        float returnTorque3(float diff);
        float returnTorque4(float diff);
        float returnTorque5(float diff);
        float returnTorque6(float diff);
        float returnTorque7(float diff);

        float calculateTorque(float diff, int jointIndex);
};

#endif