#include "Pid/TurnPid.h"

TurnPid::TurnPid(double Kp, double Ki, double Kd, double errorThreshold, double maxOutput) : Pid(Kp, Ki, Kd, maxOutput), errorThreshold(errorThreshold){};

// should return a controller input between -maxOutput and maxOutput
double TurnPid::calculateInput(double currentState, double targetState, double dt)
{
	double pidError = targetState - currentState;
	if(std::abs(pidError) < errorThreshold){
		return 0;
	}

	if (pidError > 180 * DEG_TO_RAD){
		pidError -= 360 * DEG_TO_RAD;
	}

	if (pidError < -180 * DEG_TO_RAD){
		pidError += 360 * DEG_TO_RAD;
	}
	
	integralError += (pidError * dt);

	double porportional = gains[P] * pidError;
	double integral = gains[I] * (integralError);
	double derivative = gains[D] * (pidError - prevError) / dt;

	prevError = pidError;

	double output = porportional + integral + derivative;

	return std::min(std::max(output, -maxOutput), maxOutput);
}

const char* TurnPid::getName() const{
	return "Turn";
}