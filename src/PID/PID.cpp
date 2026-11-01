#include "PID/PID.h"

PID::PID()
{
}

void PID::reset(){
	prevError = 0;
	totalTime = 0;
	integralError = 0;
}

// should return a straight direction controller input (Left Stick only) between -1.0 and 1.0
double PID::calculateInput(double velocity, double dt)
{
	double pidError = TARGET_VELOCITY - velocity;
	integralError += (pidError * dt);

	double porportional = Kp * pidError;
	double integral = Ki * (integralError);
	double derivative = Kd * (pidError - prevError) / dt;

	prevError = pidError;

	return porportional + integral + derivative;
}
