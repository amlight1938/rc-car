#include "PID/PID.h"

PID::PID()
{
	prevError = 0;
	totalTime = 0;
	integralError = 0;
}

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
