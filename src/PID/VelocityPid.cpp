#include "Pid/VelocityPid.h"

VelocityPid::VelocityPid(double Kp, double Ki, double Kd, double maxOutput) : Pid(Kp, Ki, Kd, maxOutput){};

const char* VelocityPid::getName() const{
	return "VelPid";
}
