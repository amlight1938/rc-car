## Overview

## Skills developed
- Kalman Filtering, Extended Kalman Filtering
- PID control
- Embedded software, RTOS, prioritized control tasks
- UDP, remote telemetry logging
- State machine design
- Circuitry, circuit schematic design, 3D modeling

## Features
- Autonomous return to home navigation with EKF state estimation and PID controller, Sense -> Estimate -> Control loop
- Remote control manual operation



## Design Considerations
#### Remote Logging
- Memory / speed considerations
- Queue + singleton architecture
- Separate lower priority task

#### State machine, handles fault tolerance
- Manual remote control
- Autonomous return to home navigation with EKF state estimation and PID controller, Sense -> Estimate -> Control loop
- Bluetooth pairing
- Sensor calibration
- Disarmed -> safety critical design philosphy
 
#### Embedded software design
- Seperate prioritized RTOS tasks for logging and control
	- Allow task prioritization
	- Deterministic fixed timing loop control task for EKF and PID

#### Extended Kalman Filter
- The following state vector $\bar{x}$ and differential drive physics model was defined to be:
$$
	\begin{aligned}
	&\bar{x} = [x,\ y,\ v,\ a,\ \theta,\ \omega]^T \\[1em]
	&x_{k+1} = x_k + \left(v_k\,\Delta t + \tfrac{1}{2}a_k\,\Delta t^2\right)\cos(\theta_k) \\
	&y_{k+1} = y_k + \left(v_k\,\Delta t + \tfrac{1}{2}a_k\,\Delta t^2\right)\sin(\theta_k) \\
	&v_{k+1} = v_k + a_k\,\Delta t \\
	&a_{k+1} = a_k \\
	&\theta_{k+1} = \theta_k + \omega_k\,\Delta t \\
	&\omega_{k+1} = \omega_k
	\end{aligned}
	$$

#### PID controllers
- Abstract base class

#### 3D modeling / Physics
- Differential drive mechanics





## Verification and Validation





## Difficult bugs/challenges overcome
- Wheel slippage




## Project Limitations
- IMU magnetometer
- Limited sensors, integration error buildup



## Future Work



## Design Directory

[Schematic](Design/Schematic)

[Parts](Design/PartsList)

[Third Party Library Information](Design/ThirdPartyLibInfo)