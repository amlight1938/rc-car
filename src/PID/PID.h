#pragma once
#include <Arduino.h>

class Pid
{
public:
	void reset();

	virtual double calculateInput(double currentState, double targetState, double dt);

	void toggleGainToAdjust();
	void decrementGain();
	void incrementGain();

	double getKp() const;
	double getKi() const;
	double getKd() const;

	virtual const char *getName() const = 0;

	virtual ~Pid() = default; // destructor required for inheritance cleanup

protected:
	Pid(double Kp, double Ki, double Kd, double maxOutput); // ctor protected so we cant create a Pid object (must create an object of its child classes types)

	static const int P = 0, I = 1, D = 2;

	double gains[3]; // Kp, Ki, Kd

	double prevError = 0;
	double totalTime = 0;
	double integralError = 0;

	const double maxOutput; // should be between 0 and 1 (inclusive of 1)

	int currentToggleGain = 0;
};