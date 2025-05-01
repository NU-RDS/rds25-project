#include "position_control.hpp"

PositionControl::forceControl(float ff, float kp, float ki, float kd, float ks) :
	Ff(ff), Kp(kp), Ki(ki), Kd(kd), Ks(ks) {}

float PositionControl::encoderToForce(Encoder encoder)
{
	float angle = encoder.readEncoderDeg();
	float distance = R_ENCODER_PULLEY * (angle / 180 * _PI);
	return distance * this->Ks;
}

void PositionControl::forceGeneration(ForceType forceType, int t) {
	switch (forceType) {
		case ForceType::STEP:
			// Step input
			this->referenceForce = (t % 2 == 0) ? 1.0f : 3.0f;
			break;
			
		case ForceType::SIN:
			// Sinusoidal force (2 + 1 sin(2π f t))N
			{
				float frequency = 1.0f;
				this->referenceForce = 2.0f + 1.0f * sin(2.0f * _PI * frequency * t); 
			}
			break;
			
		case ForceType::MAX:
			// Maximum continuous force
			this->referenceForce = MAX_TENDON_FORCE; 
			break;
			
		case ForceType::TENDON_MAX:
			// 50% of maximum continuous force for tendon stretching test 1
			this->referenceForce = MAX_TENDON_FORCE/2; 
			break;
			
		case ForceType::TENDON_SIN:
			// Sinusoidal force between 10% and 50% of maximum continuous force at 1 Hz
			{
				this->referenceForce = (0.3f * MAX_TENDON_FORCE) + (0.2f * MAX_TENDON_FORCE * sin(2.0f * _PI * 1.0f * t));
			}
			break;
	}
}

float PositionControl::forcePID(int encoderCS, ForceType forceType)
{
    static float prevErr = 0.0f;
    static float intErr = 0.0f;
    static int timeStep = 0;

    // generate reference force
    forceGeneration(forceType, timeStep++);
    
    // current force from the encoder
    Encoder encoder(encoderCS, 1);
    float currentForce = encoderToForce(encoder);
    
    // error
    float error = this->referenceForce - currentForce;
    intErr += error;
    
    // anti-windup
    if (intErr > ANTI_WINDUP_F) {
        intErr = ANTI_WINDUP_F;
    } else if (intErr < -ANTI_WINDUP_F) {
        intErr = -ANTI_WINDUP_F;
    }
    
    // derivative
    float dErr = error - prevErr;
    prevErr = error;
    
    // PID
    resultantForce = Ff * this->referenceForce + 
                     Kp * error + 
                     Ki * intErr + 
                     Kd * dErr;
    
    
    // if (resultantForce > MAX_TENDON_FORCE) {
    //     resultantForce = MAX_TENDON_FORCE;
    // } else if (resultantForce < 0.0f) {
    //     resultantForce = 0.0f; 
    // }
    
    // TODO: NULL SPACE
    
    float motorTorque = R_ENCODER_PULLEY * resultantForce;
    return resultantForce;
}

void PositionControl::positionPrint()
{
	Serial.print(reference);
    Serial.print(",");
    Serial.println(PIDresult);
	delay(10);
}