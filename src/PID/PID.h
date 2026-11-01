#pragma once
#include <string>

class PID
{
public:
	PID();

	void reset();

	double calculateInput(double velocity, double dt);

	void decrementKd();
	void incrementKp();
	void decrementKp();
	void incrementKi();
	void decrementKi();
	void incrementKd();

	double getKp();
	double getKi();
	double getKd();

private:
	const double TARGET_VELOCITY = 0.3;

	double Kp = 0.7;
	double Ki = 6.4;
	double Kd = 0.0;

	double prevError = 0;
	double totalTime = 0;
	double integralError = 0;
};