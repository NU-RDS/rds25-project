#include <Arduino.h>

#include <string>

#include "ODriveCan.hpp"
#include "StateManager.hpp"
#include "comms.hpp"

StateManager state_manager;
std::string serialCommand = "";

static bool __canInfSendMessage(void* intf, uint32_t id, uint8_t length, const uint8_t* data);
static void __canInfPumpEvents(void* inft);

static ODriveCanIntfWrapper canWrapper = {
    .send_msg_ = __canInfSendMessage,
    .pump_events_ = __canInfPumpEvents};

const int NUM_DRIVES = 2;
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
    float encoder_offset;
    bool offset_captured;
};

static std::vector<int> odriveIDs = {5, 6};
static ODriveControl odrives[NUM_DRIVES] =
    {{ODriveCAN(canWrapper, 5), ODriveUserData(), false, 0.0f, 0.0f, false},
     {ODriveCAN(canWrapper, 6), ODriveUserData(), false, 0.0f, 0.0f, false}};

comms::TeensyCANDriver<2, comms::CANBaudRate::CBR_250KBPS> canDriver;
comms::CommsController commsController{canDriver, comms::MCUID::MCU_HIGH_LEVEL};

static bool setupCan();
static void setupODrive(int index);
static void captureEncoderOffsets();
static void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
static void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
static void onCanMessage(const CAN_message_t& msg);
static void updateEncoderPositions();
static void updateEncoderOffsets();

void setup() {
    Serial.begin(9600);
    Serial.println("[HIGH]");
    while (!Serial) delay(100);

    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    // Initialize all ODrives
    for (int i = 0; i < NUM_DRIVES; i++) {
        Serial.print("Initializing ODrive ");
        Serial.println(i);
        setupODrive(i);
    }

    captureEncoderOffsets();

    state_manager.initialize();
}

/**
 * @brief Main control loop for the system, intended to be called repeatedly.
 *
 * This function manages periodic control tasks at a fixed rate (100Hz).
 * It performs the following operations:
 *   - Handles communication controller tick.
 *   - Updates the system state based on serial commands.
 *   - Updates encoder positions.
 *   - Retrieves and prints current and desired joint angles (pitch and yaw) in a GUI-parsable format.
 *   - Executes the main control loop for the state manager.
 *   - Computes and clamps control signals for pitch and yaw motors.
 *   - Sets motor torque and running status (actual torque commands are commented out).
 *   - Prints control signals for debugging.
 *
 * Timing is managed using a static variable to ensure actions occur at the specified control period.
 */
void loop() {
    static unsigned long lastControlTime = 0;
    const unsigned long CONTROL_PERIOD_MS = 10;  // 100Hz control rate

    commsController.tick();

    if (millis() - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = millis();

        state_manager.updateState(serialCommand);

        // Get_Encoder_Estimates_msg_t encoder = odrives[1].user_data.last_feedback;
        // motor ang is motor shaft ang
        // Serial.print("Motor ");
        // Serial.print(1);
        // Serial.print(" is at ");
        // Serial.println(state_manager.getWrist()->getPitch()->getMotorValue());

        // encoder = odrives[0].user_data.last_feedback;
        // motor ang is motor shaft ang
        // Serial.print("Motor ");
        // Serial.print(0);
        // Serial.print(" is at ");
        // Serial.println(state_manager.getWrist()->getYaw()->getMotorValue());

        updateEncoderPositions();

        // Get current and desired positions
        double pitch_desired = state_manager.getWrist()->getPitch()->getDesiredPosition() * 180.0 / M_PI;  // rad to deg
        double pitch_current = state_manager.getWrist()->getPitch()->getCurrentPosition() * 180.0 / M_PI;  // rad to deg
        double yaw_desired = state_manager.getWrist()->getYaw()->getDesiredPosition() * 180.0 / M_PI;      // rad to deg
        double yaw_current = state_manager.getWrist()->getYaw()->getCurrentPosition() * 180.0 / M_PI;      // rad to deg
        
        // Print in format that GUI can parse
        Serial.print("JOINT_ANGLES: pitch_des=");
        Serial.print(pitch_desired, 2);
        Serial.print(", pitch_cur=");
        Serial.print(pitch_current, 2);
        Serial.print(", yaw_des=");
        Serial.print(yaw_desired, 2);
        Serial.print(", yaw_cur=");
        Serial.println(yaw_current, 2);

        state_manager.controlLoop();

        odrives[1].current_torque = 0.5f;
        odrives[1].is_running = true;
        float pitch_control = state_manager.getWrist()->getPitch()->getControlSignal();
        if (pitch_control > 1.0f) {
            pitch_control = 1.0f;
        }
        if (pitch_control < -1.0f) {
            pitch_control = -1.0f;
        }
        // odrives[1].drive.setTorque(pitch_control);
        Serial.println(pitch_control);

        odrives[0].current_torque = 0.5f;
        odrives[0].is_running = true;
        float yaw_control = state_manager.getWrist()->getYaw()->getControlSignal();
        if (yaw_control > 1.0f) {
            yaw_control = 1.0f;
        }
        if (yaw_control < -1.0f) {
            yaw_control = -1.0f;
        }
        Serial.println(yaw_control);
        // odrives[0].drive.setTorque(yaw_control);
    }
}

static bool setupCan() {
    commsController.initialize();

    commsController.setUnregisteredMessageHandler(
        [](comms::RawCommsMessage msg) {
            CAN_message_t newMsg;
            newMsg.len = msg.length;
            newMsg.id = msg.id;
            memcpy(newMsg.buf, &msg.payload, 8);
            onCanMessage(newMsg);
        });

    return true;
}

static bool __canInfSendMessage(void* intf, uint32_t id, uint8_t length, const uint8_t* data) {
    comms::RawCommsMessage msg;
    msg.id = id;
    msg.length = length;
    memcpy(&msg.payload, data, length);
    canDriver.sendMessage(msg);
    return true;
}

static void __canInfPumpEvents(void* inft) {
    // no ops
}

static void setupODrive(int index) {
    // Register callbacks
    odrives[index].drive.onFeedback(onFeedback, &odrives[index].user_data);
    odrives[index].drive.onStatus(onHeartbeat, &odrives[index].user_data);
    // odrives[index].drive.getCurrent(getCurrents, &odrives[index].user_data);

    // Set control mode to torque control
    odrives[index].drive.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
                                           ODriveInputMode::INPUT_MODE_PASSTHROUGH);

    // Enable closed loop control
    // while (odrives[index].user_data.last_heartbeat.Axis_State !=
    //        ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //     odrives[index].drive.clearErrors();
    //     delay(1);
    //     odrives[index].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    //     for (int i = 0; i < 15; ++i) {
    //         delay(10);
    //         pumpEvents(can_intf);
    //     }
    // }
}

static void onCanMessage(const CAN_message_t& msg) {
    for (int i = 0; i < NUM_DRIVES; i++) {
        onReceive(msg, odrives[i].drive);
    }
}

static void captureEncoderOffsets() {
    Serial.println("Capturing encoder offsets...");

    // Wait for initial encoder readings
    unsigned long timeout = millis() + 5000;  // 5 second timeout

    updateEncoderOffsets();

    // for (int i = 0; i < NUM_DRIVES; i++) {
    //     odrives[i].offset_captured = false;

    //     while (!odrives[i].user_data.received_feedback && millis() < timeout) {
    //         pumpEvents(can_intf);
    //         delay(10);
    //     }

    //     if (odrives[i].user_data.received_feedback) {
    //         // Capture the initial encoder position as offset
    //         Get_Encoder_Estimates_msg_t encoder = odrives[i].user_data.last_feedback;
    //         odrives[i].encoder_offset = encoder.Pos_Estimate;  // Store raw encoder value
    //         odrives[i].offset_captured = true;

    //         Serial.print("ODrive ");
    //         Serial.print(i);
    //         Serial.print(" offset captured: ");
    //         Serial.println(odrives[i].encoder_offset);
    //     } else {
    //         Serial.print("Failed to capture offset for ODrive ");
    //         Serial.println(i);
    //     }
    // }
}

static void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;
}

static void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
}

static void updateEncoderPositions() {
    static std::map<uint8_t, std::function<void(double)>> palmSetterFunctions = {
        {6, [](double value) { state_manager.getWrist()->getYaw()->setCurrentPosition(value - state_manager.getWrist()->getYaw()->getEncoderOffset()); }}};

    for (auto pair : palmSetterFunctions) {
        comms::Option<float> encoderValueOpt = commsController.getSensorValue(
            comms::MCUID::MCU_PALM, pair.first);

        if (encoderValueOpt.isNone()) {
            continue;
        }

        Serial.printf("Encoder %d, value %f\n", pair.first, encoderValueOpt.value());
        pair.second(encoderValueOpt.value());
    }
}

static void updateEncoderOffsets() {
    static std::map<uint8_t, std::function<void(double)>> palmSetterFunctions = {
        {6, [](double value) { state_manager.getWrist()->getYaw()->setCurrentPosition(value * M_PI / 180); }}
        // {6, [](double value) { state_manager.getWrist()->getYaw()->setEncoderOffset(value); }}
    };

    for (auto pair : palmSetterFunctions) {
        comms::Option<float> encoderValueOpt = commsController.getSensorValue(
            comms::MCUID::MCU_PALM, pair.first);

        if (encoderValueOpt.isNone()) {
            continue;
        }
        
        pair.second(encoderValueOpt.value());
    }
}