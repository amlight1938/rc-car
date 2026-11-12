#pragma once
#include "Pid.h"

class TurnPid : public Pid {
	public:
		TurnPid(double Kp, double Ki, double Kd, double errorThreshold, double maxOutput = 1.0);
		
		double calculateInput(double currentState, double targetState, double dt) override;

		const char* getName() const override;

	private:
		const double errorThreshold;
};
