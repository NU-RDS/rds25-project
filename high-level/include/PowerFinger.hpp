#ifndef POWER_FINGER_HPP
#define POWER_FINGER_HPP

#include <Arduino.h>
#include <memory>
#include <string>
#include <unordered_map>

#include "Joint.hpp"
#include "RDS_constants.hpp"

/**
 * @class PowerFinger
 * @brief Represents a robotic finger with power grasping capabilities.
 *
 * The PowerFinger class encapsulates the kinematics and control logic for a single robotic finger,
 * providing methods to set joint angles, send torque commands, and retrieve joint information.
 */
 
/**
 * @brief Default constructor for PowerFinger.
 */
 
/**
 * @brief Computes the kinematics for the finger.
 */
 
/**
 * @brief Sets the desired joint angles for the grasp.
 * @param grasp_desired The desired angle for the grasp joint.
 */
 
/**
 * @brief Sends a torque command to the grasp joint.
 * @param T_grasp The torque value to be applied to the grasp joint.
 */
 
/**
 * @brief Retrieves a pointer to the grasp joint.
 * @return Pointer to the Joint object representing the grasp joint.
 */
 
/**
 * @brief Gets the desired joint angles for the finger.
 * @return An unordered_map mapping joint names to their desired angles.
 */
 
/**
 * @brief Gets the current joint angles for the finger.
 * @return An unordered_map mapping joint names to their current angles.
 */
 
/**
 * @brief Calculates the control output for the finger.
 * @return The computed control value (e.g., torque or force).
 */
class PowerFinger {
    private:
        std::unique_ptr<Joint> Grasp;

        double J_pow = 1.0f;

    public:
        PowerFinger();
        ~PowerFinger() = default;
        
        void kinematics();

        void setJointAngles(double grasp_desired);
        void sendTorqueCommand(double T_grasp);
        Joint* getGrasp() { return Grasp.get(); }

        const std::unordered_map<std::string, double> getDesiredJointAngles();
        const std::unordered_map<std::string, double> getCurrentJointAngles();
        double calculateControl();
};

#endif // POWER_FINGER_HPP