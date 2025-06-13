/**
 * @file StateManager.hpp
 * @brief Defines the StateManager class for managing the state and control of a robotic hand system.
 *
 * The StateManager class encapsulates the logic for interfacing with various subsystems of a robotic hand,
 * including the wrist, dexterous finger, power finger, and tendon kinematics. It manages communication with
 * the microcontroller unit (MCU), tracks system timing, and controls the movement phases of the device.
 *
 * @author 
 * @date 
 */

 /**
    * @enum MovementPhase
    * @brief Represents the different phases of movement for the robotic system.
    * - IDLE: System is idle and not performing any movement.
    * - ERROR: An error has occurred in the system.
    * - MOVING: The system is currently executing a movement.
    * - COMPLETE: The movement has been completed.
    */

/**
 * @class StateManager
 * @brief Manages the state, control, and communication for the robotic hand system.
 *
 * The StateManager class is responsible for:
 * - Initializing and managing subsystem objects (Wrist, DexterousFinger, PowerFinger, TendonKinematics).
 * - Handling communication with the MCU over a serial port.
 * - Tracking timing and time steps for control loops.
 * - Managing the current movement phase and sequencing of movements.
 * - Providing access to subsystem objects and joint position data.
 * - Executing control loops and updating the system state based on serial commands.
 *
 * Public Methods:
 * - StateManager(): Constructor.
 * - ~StateManager(): Destructor.
 * - bool initialize(): Initializes the system and subsystems.
 * - bool connectToMCU(): Establishes communication with the MCU.
 * - void updateState(std::string& serialCommand): Updates the system state based on a serial command.
 * - void updateGUI(): Updates the graphical user interface with current state information.
 * - void controlLoop(): Executes the main control loop for the system.
 * - Wrist* getWrist(): Returns a pointer to the Wrist subsystem.
 * - DexterousFinger* getDexFinger(): Returns a pointer to the DexterousFinger subsystem.
 * - PowerFinger* getPowFinger(): Returns a pointer to the PowerFinger subsystem.
 * - TendonKinematics* getKinematics(): Returns a pointer to the TendonKinematics subsystem.
 * - void setJointPositions(...): Sets the desired positions for all joints.
 * - std::unordered_map<std::string, double> getCurrentJointPositions(): Gets the current joint positions.
 * - std::unordered_map<std::string, double> getDesiredJointPositions(): Gets the desired joint positions.
 * - void executeSequencedMovement(): Executes a predefined sequence of movements.
 * - void stopMovement(): Stops any ongoing movement.
 * - void pauseMovement(): Pauses the current movement.
 * - void resumeMovement(): Resumes a paused movement.
 * - void sendTorqueCommands(): Sends torque commands to the actuators.
 * - void setMovementPhase(MovementPhase phase): Sets the current movement phase.
 * - MovementPhase getMovementPhase() const: Gets the current movement phase.
 * - bool isMovementComplete() const: Checks if the movement phase is COMPLETE.
 */
#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <Arduino.h>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include "SerialHelpers.hpp"
#include "TendonKinematics.hpp"
#include "PowerFinger.hpp"
#include "DexterousFinger.hpp"
#include "Wrist.hpp"

enum MovementPhase {
    IDLE,
    ERROR,
    MOVING,
    COMPLETE
};

class StateManager {
    private:
        std::unique_ptr<Wrist> wrist;
        std::unique_ptr<DexterousFinger> dexFinger;
        std::unique_ptr<PowerFinger> powFinger;
        std::unique_ptr<TendonKinematics> kinematics;
        
        // Communication with MCU
        bool connected;
        std::string mcuPort;
        
        // Timing
        std::chrono::time_point<std::chrono::steady_clock> lastUpdateTime;
        double dt; // Time step in seconds

        MovementPhase currentMovementPhase;
    
    public:
        StateManager();
        ~StateManager() = default;
        
        // Basic system functions
        bool initialize();
        bool connectToMCU();
        void updateState(std::string& serialCommand);
        void updateGUI();

        void controlLoop();

        // Subsystem access
        Wrist* getWrist() { return wrist.get(); }
        DexterousFinger* getDexFinger() { return dexFinger.get(); }
        PowerFinger* getPowFinger() { return powFinger.get(); }
        TendonKinematics* getKinematics() {return kinematics.get(); }

        // Joint position setting
        void setJointPositions(double wristPitch, double wristYaw, double dexPip, double dexDip, double dexMcp, double dexSplay, double powGrasp);

        // Get current and desired joint positions
        std::unordered_map<std::string, double> getCurrentJointPositions();
        std::unordered_map<std::string, double> getDesiredJointPositions();

        // Movement control and sequencing
        void executeSequencedMovement();
        void stopMovement();
        void pauseMovement();
        void resumeMovement();
        void sendTorqueCommands();
        void setMovementPhase(MovementPhase phase);
        MovementPhase getMovementPhase() const { return currentMovementPhase; }
        bool isMovementComplete() const { return currentMovementPhase == COMPLETE; }
};

#endif //STATE_MANAGER_HPP