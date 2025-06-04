#include "ForceControl.hpp"

ForceControl::ForceControl(float ff, float kp, float ki, float kd, float ks) :
	Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks) {}

float ForceControl::encoderToForce(Encoder encoder)
{
	float angle = encoder.readEncoderDeg();
	float distance = R_ENCODERPULLEY * (angle / 180 * _PI);
	return distance * this->Ks;
}

void ForceControl::forceGeneration(ForceType forceType, int t) {
	switch (forceType) {
		case ForceType::STEP:
			expectedForce = (t % 2 == 0) ? 1.0f : 3.0f;
			break;
			
		case ForceType::SIN:
			// Sinusoidal force (2 + 1 sin(2π f t))N
			{
				float frequency = 1.0f;
				expectedForce = 2.0f + 1.0f * sin(2.0f * _PI * frequency * t); 
			}
			break;
			
		case ForceType::MAX:
			// Maximum continuous force
			expectedForce = 50.0f; // Example value, should be determined experimentally
			break;
			
		case ForceType::TENDON_MAX:
			// 50% of maximum continuous force for tendon stretching test 1
			expectedForce = 25.0f; // Assuming max is 50N, this should be 50% of MAX
			break;
			
		case ForceType::TENDON_SIN:
			// Sinusoidal force between 10% and 50% of maximum continuous force at 1 Hz
			{
				float maxForce = 50.0f; // Should be determined experimentally
				expectedForce = (0.3f * maxForce) + (0.2f * maxForce * sin(2.0f * _PI * 1.0f * t / 1000.0f));
				// This gives a sinusoidal force between 10% (0.3-0.2) and 50% (0.3+0.2) of max force
			}
			break;
	}
}

void ForceControl::forcePID(int forcetype)
{
	
}