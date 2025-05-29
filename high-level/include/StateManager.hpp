#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <Arduino.h>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include "PowerFinger.hpp"
#include "DexterousFinger.hpp"
#include "Wrist.hpp"
#include "comms.hpp"

enum MovementPhase {
    IDLE,
    PAUSED,
    WRIST_MOVEMENT,
    FINGER_MOVEMENT,
    COMPLETE
};

class StateManager {
    private:
        std::unique_ptr<Wrist> wrist;
        std::unique_ptr<DexterousFinger> dexFinger;
        std::unique_ptr<PowerFinger> powFinger;
        
        // Communication with MCU
        bool connected;
        std::string mcuPort;
        
        // Timing
        std::chrono::time_point<std::chrono::steady_clock> lastUpdateTime;
        double dt; // Time step in seconds

        MovementPhase currentMovementPhase;

        std::function<void(comms::MCUID)> _heartbeatCallback = nullptr;
    
    public:
        StateManager();
        ~StateManager() = default;
        
        // Basic system functions
        bool initialize();
        bool connectToMCU();
        void updateState();
        void updateGUI();

        void controlLoop();
        void processMessage(const comms::MessageInfo& message_info, const comms::RawCommsMessage& message_raw);

        // Subsystem access
        Wrist* getWrist() { return wrist.get(); }
        DexterousFinger* getDexFinger() { return dexFinger.get(); }
        PowerFinger* getPowFinger() { return powFinger.get(); }

        // Joint position setting
        void setJointPositions(double wristRoll, double wristPitch, double wristYaw, double dexPip, double dexDip, double dexMcp, double dexSplain, double powGrasp);

        // Get current and desired joint positions
        std::unordered_map<std::string, double> getCurrentJointPositions();
        std::unordered_map<std::string, double> getDesiredJointPositions();

        // Movement control and sequencing
        void executeSequencedMovement();
        void stopMovement();
        void pauseMovement();
        void resumeMovement();
        void sendTorqueCommands();
        MovementPhase getMovementPhase() const { return currentMovementPhase; }
        bool isMovementComplete() const { return currentMovementPhase == COMPLETE; }
};

#endif //STATE_MANAGER_HPP