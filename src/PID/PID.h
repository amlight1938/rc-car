#pragma once

class PID
{
public:
	PID();

	double calculateInput(double velocity, double dt);

private:
	const double TARGET_VELOCITY = 0.3;
	const double Kp = 1;
	const double Ki = 1;
	const double Kd = 1;

	double prevError;
	double totalTime;
	double integralError;
};