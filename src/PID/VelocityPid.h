#pragma once
#include "Pid.h"

class VelocityPid : public Pid {
	public:

		/**
		 * \param Kp proportional gain
		 * \param Ki integral gain
		 * \param Kd derivative gain
		 * \param maxOutput bounds the output of the PID and therefore input to car.control() to [-maxOutput, maxOutput]
		 */
		VelocityPid(double Kp, double Ki, double Kd, double maxOutput = 1.0);

		const char* getName() const override;
};
