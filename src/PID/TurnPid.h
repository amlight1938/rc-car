#pragma once
#include "Pid.h"

class TurnPid : public Pid {
	public:

		/**
		 * \param Kp proportional gain
		 * \param Ki integral gain
		 * \param Kd derivative gain
		 * \param errorThreshold if the abs(error) < errorThreshold then the PID will return 0 output -> dont turn the car because we're at the proper heading
		 * \param maxOutput bounds the output of the PID and therefore input to car.control() to [-maxOutput, maxOutput]
		 */
		TurnPid(double Kp, double Ki, double Kd, double errorThreshold, double maxOutput = 1.0);
		
		double calculateInput(double currentState, double targetState, double dt) override;

		const char* getName() const override;

	private:
		const double errorThreshold;
};
