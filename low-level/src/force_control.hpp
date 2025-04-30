#ifndef FORCE_CONTROL_HPP
#define FORCE_CONTROL_HPP

#include "encoder.hpp"

#define R_ENCODER_PULLEY 1
#define _PI 3.1415
#define MAX_TENDON_FORCE 10
#define ENCODER1_CS 10
#define ANTI_WINDUP_F 10

class ForceControl
{
private:
	float Ff; // feedforwad
	float Kp; // proportional
	float Ki; // integral
	float Kd; // derivative

	float Ks; // spring constant

	float referenceForce; // input force 
	float resultantForce; // after PID

	float encoderToForce(Encoder encoder); // read from encoder and calculate tendon force
	void forceGeneration(ForceType forceType, int t); // t for time in sin force function

public:
	// type of force given in the instruction
	enum class ForceType
	{
		STEP,
		SIN,
		MAX,
		TENDON_MAX,
		TENDON_SIN
	}

	forceControl(float ff, float kp, float ki, float kd, float ks);

	void forcePID(ForceType forcetType);
	void forcePrint();
};


#endif // FORCE_CONTROL_HPP