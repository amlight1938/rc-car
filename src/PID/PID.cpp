#include "Pid/Pid.h"

Pid::Pid(double Kp, double Ki, double Kd, double maxOutput) : gains{Kp, Ki, Kd}, maxOutput(maxOutput)
{
}

void Pid::reset()
{
	prevError = 0;
	totalTime = 0;
	integralError = 0;
}

// should return a controller input between -maxOutput and maxOutput
double Pid::calculateInput(double currentState, double targetState, double dt)
{
	double pidError = targetState - currentState;
	
	integralError += (pidError * dt);

	double porportional = gains[P] * pidError;
	double integral = gains[I] * (integralError);
	double derivative = gains[D] * (pidError - prevError) / dt;

	prevError = pidError;

	double output = porportional + integral + derivative;

	return std::min(std::max(output, -maxOutput), maxOutput);
}

void Pid::toggleGainToAdjust(){
	currentToggleGain = (currentToggleGain + 1) % 3;
}

void Pid::incrementGain()
{
	gains[currentToggleGain] += 0.1; 
}

void Pid::decrementGain()
{
	gains[currentToggleGain] -= 0.1; 
}

double Pid::getKp()
{
	return gains[P];
}

double Pid::getKi()
{
	return gains[I];
}

double Pid::getKd()
{
	return gains[D];
}