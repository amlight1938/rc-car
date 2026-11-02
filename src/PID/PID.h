#pragma once

class PID
{
public:
	PID();

	void reset();

	double calculateInput(double velocity, double dt);

private:
	const double TARGET_VELOCITY = 0.3;
	const double Kp = 3;
	const double Ki = 1.0;
	const double Kd = 0.0;

	double prevError = 0;
	double totalTime = 0;
	double integralError = 0;
};