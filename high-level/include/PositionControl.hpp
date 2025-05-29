#ifndef POSITIONCONTROL_HPP
#define POSITIONCONTROL_HPP

#include <Arduino.h>
#include <map>

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
		SIN
	};

	PositionControl(float ff, float kp, float kd, float ks);

	float positionPD(int encoderCS, int encoderId, PositionType forceType);
	void positionPrint();

    // Getters
    float getJointAngle(Encoder& encoder);
    float getReferencePosition() { return referencePosition; }
    float getPidPosition() { return resultantPosition; }
    float nullspaceConversion();

private:
	float Ff; // feedforwad
	float Kp; // proportional
	float Ki; // integral
	float Kd; // derivative
    float Ks;

	float referencePosition; // input joint angle 
	float resultantPosition; // after PID

	PositionType posType;

	void positionGeneration(PositionType positionType, int t); // t for time in sin force function
};

#endif // POSITION_CONTROL_HPP