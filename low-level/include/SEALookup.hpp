/**
 * @file SEALookup.hpp
 * @brief Declaration of the SEALookup class for torque lookup operations.
 *
 * The SEALookup class provides methods to compute and retrieve torque values
 * based on a given difference value (`diff`) for up to seven different joints.
 * Each joint has a dedicated method to return its corresponding torque value.
 * Additionally, a generic method is provided to calculate the torque for a
 * specified joint index.
 */

 /**
    * @class SEALookup
    * @brief Provides torque lookup functionality for multiple joints.
    *
    * This class contains methods to return torque values for seven different joints,
    * as well as a generic method to calculate torque based on joint index.
    */
 
 /**
    * @brief Default constructor for SEALookup.
    */
 
 /**
    * @brief Returns the torque value for SEAs based on the input difference.
    * @param diff The difference value used for torque calculation.
    * @return The computed torque for all SEAs
    * 
 /**
    * @brief Calculates the torque for a specified joint index based on the input difference.
    * @param diff The difference value used for torque calculation.
    * @param jointIndex The index of the joint (1-7).
    * @return The computed torque for the specified joint.
    */
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