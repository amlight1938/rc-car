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
- Remote Logging
	- Memory / speed considerations
	- Queue + singleton architecture
	- Separate lower priority task

- State machine, handles fault tolerance
	- Manual remote control
	- Autonomous return to home navigation with EKF state estimation and PID controller, Sense -> Estimate -> Control loop
	- Bluetooth pairing
	- Sensor calibration
	- Disarmed -> safety critical design philosphy
 
- Embedded software design
	- Seperate prioritized RTOS tasks for logging and control
		- Allow task prioritization
		- Deterministic fixed timing loop control task for EKF and PID

- Extended Kalman Filter
	- $\vec{x}$ = [x, y, v, a, $\theta$, $\omega$]

- PID controllers
	- Abstract base class

- 3D modeling / Physics
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