#ifndef FORCECONTROL_HPP
#define FORCECONTROL_HPP

#include "Encoder.hpp"

#define R_ENCODERPULLEY 1
#define _PI 3.1415

class ForceControl {
	public:
		// type of force given in the instruction
		enum class ForceType
		{
			STEP,
			SIN,
			MAX,
			TENDON_MAX,
			TENDON_SIN
		};

		ForceControl(float ff, float kp, float ki, float kd, float ks);

	void forcePID(int forcetype);

	private:
		float Ff;
		float Kp; 
		float Ki;
		float Kd;

		float Ks; // spring constant

		float expectedForce;
		float resultantForce;

		float encoderToForce(Encoder encoder); // read from encoder and calculate tendon force
		void forceGeneration(ForceType forceType, int t); // t for time in sin force function
};


#endif // FORCE_CONTROL_HPP