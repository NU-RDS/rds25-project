#ifndef POSITION_CONTROL_HPP
#define POSITION_CONTROL_HPP

#include "encoder.hpp"

class PositionControl
{
    private:
	float Ff; // feedforwad
	float Kp; // proportional
	float Ki; // integral
	float Kd; // derivative

	float reference; // input joint angle 
	float PIDresult; // after PID

	void positionGeneration(PositionType positionType, int t); // t for time in sin force function

public:
	// type of force given in the instruction
	enum class PositionType
	{
		STEP,
		SIN
	}

	positionControl(float ff, float kp, float ki, float kd, float ks);

	void positionPID(PositionType positionType);
	void positionPrint();
}

#endif // POSITION_CONTROL_HPP