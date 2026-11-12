#pragma once
#include "Pid.h"

class VelocityPid : public Pid {
	public:
		VelocityPid(double Kp, double Ki, double Kd, double maxOutput = 1.0);

		const char* getName() const override;
};
