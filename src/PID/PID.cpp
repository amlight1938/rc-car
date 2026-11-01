#include "PID/PID.h"

PID::PID()
{
}

void PID::reset()
{
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

void PID::incrementKp()
{
	Kp = Kp + 0.1;
}

void PID::decrementKp()
{
	Kp = Kp - 0.1;
}

void PID::incrementKi()
{
	Ki = Ki + 0.1;
}

void PID::decrementKi()
{
	Ki = Ki - 0.1;
}

void PID::incrementKd()
{
	Kd = Kd + 0.1;
}

void PID::decrementKd()
{
	Kd = Kd - 0.1;
}

double PID::getKp()
{
	return Kp;
}

double PID::getKi()
{
	return Ki;
}

double PID::getKd()
{
	return Kd;
}