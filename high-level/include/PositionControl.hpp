#ifndef POSITIONCONTROL_HPP
#define POSITIONCONTROL_HPP

#include <Arduino.h>
#include <map>
#include <chrono>
#include <cmath>

#include "Encoder.hpp"

#define ROM_MAX 30
#define ANTI_WINDUP_P 100

/**
 * @class PositionControl
 * @brief Implements a position control system using PID control logic.
 *
 * This class provides methods to control and generate position commands using
 * proportional, integral, and derivative (PID) gains, as well as a static gain.
 * It supports different types of position instructions, such as step, sinusoidal,
 * and manual modes.
 */
 
/**
 * @enum PositionControl::PositionType
 * @brief Enumerates the types of position instructions.
 * 
 * - STEP: Step input position.
 * - SIN: Sinusoidal input position.
 * - MANUAL: Manually specified position.
 */

/**
 * @brief Constructs a PositionControl object with specified PID and static gains.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param ks Static gain.
 */

/**
 * @brief Computes the PD (Proportional-Derivative) control output for position.
 * @param desiredPosition The target position to achieve.
 * @param currentPosition The current measured position.
 * @return The computed control output.
 */

/**
 * @brief Prints the current position control parameters and state.
 */

/**
 * @brief Generates a position command based on the specified type and time.
 * @param positionType The type of position instruction (STEP, SIN, MANUAL).
 * @param t The time parameter, used for generating sinusoidal commands.
 */
class PositionControl
{
public:
	// type of force given in the instruction
	enum class PositionType
	{
		STEP,
		SIN,
		MANUAL
	};

	PositionControl(double kp, double ki, double kd, double ks);

	double positionPD(double desiredPosition, double currentPosition);
	void positionPrint();

private:
	double Kp; // proportional
	double Ki; // integral
	double Kd; // derivative
    double Ks;

	double prevError;
	double integralError;
	double integralCap = 0.25;
	std::chrono::steady_clock::time_point prevTime;

	PositionType posType;

	void positionGeneration(PositionType positionType, int t); // t for time in sin force function
};

#endif // POSITION_CONTROL_HPP