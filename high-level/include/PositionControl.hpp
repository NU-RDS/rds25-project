#ifndef POSITIONCONTROL_HPP
#define POSITIONCONTROL_HPP

#include <Arduino.h>
#include <map>
#include <chrono>
#include <cmath>

#include "Encoder.hpp"

#define ROM_MAX 30
#define ANTI_WINDUP_P 100

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