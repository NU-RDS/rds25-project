#ifndef POSITION_CONTROL_HPP
#define POSITION_CONTROL_HPP

#include "encoder.hpp"

#define ROM_MAX 30
#define ANTI_WINDUP_P 100

class PositionControl
{
    private:
	float Ff; // feedforwad
	float Kp; // proportional
	float Ki; // integral
	float Kd; // derivative

	float referencePosition; // input joint angle 
	float pidPosition; // after PID

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

    // Getters
    float getReferencePosition() { return referencePosition; }
    float getPidForce() { return pidPosition; }
}

#endif // POSITION_CONTROL_HPP