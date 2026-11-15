## Features / Overview
- A deterministic, fault-tolerant embedded control system in C++ on a resource constrained ESP32 microcontroller with FreeRTOS priority-scheduled tasks, a fixed rate control loop, and hardware abstraction across motors, sensors, and vehicle classes
- State machine architecture that safely handles faults, state transitions, and hardware fail-safes on communication loss
- UDP telemetry pipeline for remote logging with pre-allocated queuing and a dedicated FreeRTOS task to separate logging from the time constrained, prioritized control loop 
- Extended Kalman Filter (EKF) estimates pose in real time by fusing IMU and odometry measurements
- Autonomous open-field return to home navigation capability using EKF state estimation and PID controller in a Sense -> Estimate -> Control cycle 
- Remote control manual operation via Bluetooth



PICTURES / VIDEOS GO HERE



## Skills developed
- Kalman Filtering, Extended Kalman Filtering
- PID control
- Embedded real-time control software, RTOS, prioritized control tasks
- UDP, remote telemetry logging
- State machine design
- Data processing and plotting (Pandas, Matplotlib) 
- Circuitry, circuit schematic design, 3D modeling



## Design Considerations
#### Remote Logging
- Memory / speed considerations
- Queue + singleton architecture
- Separate lower priority task
- UDP vs TCP

#### State machine, handles fault tolerance
- Manual remote control
- Autonomous return to home navigation with EKF state estimation and PID controller, Sense -> Estimate -> Control 
- Bluetooth pairing
- Sensor calibration
- Disarmed -> safety critical design philosphy
 
#### Embedded software design
- Seperate prioritized RTOS tasks for logging and control
	- Allow task prioritization
	- Deterministic fixed timing loop control task for EKF and PID
	- Hardware abstraction accross motors, sensors, and vehicle classes

#### Extended Kalman Filter
- The following state vector $\bar{x}$ and differential drive physics model was defined to be:

```math
\begin{array}{rcl}
\bar{x} & = & [x,\ y,\ v,\ a,\ \theta,\ \omega]^T \\[6pt]
x_{k+1} & = & x_k + (v_k\,\Delta t + a_k\,\Delta t^2/2)\cos(\theta_k) \\[6pt]
y_{k+1} & = & y_k + (v_k\,\Delta t + a_k\,\Delta t^2/2)\sin(\theta_k) \\[6pt]
v_{k+1} & = & v_k + a_k\,\Delta t \\[6pt]
a_{k+1} & = & a_k \\[6pt]
\theta_{k+1} & = & \theta_k + \omega_k\,\Delta t \\[6pt]
\omega_{k+1} & = & \omega_k
\end{array}
```

- This required an Extended Kalman Filter since the motion model was nonlinear

#### PID controllers
- Abstract base class




## Verification and Validation

PLOTS GO HERE




## Difficult bugs/challenges overcome
- Wheel slippage
- Ghost heading drift




## Project Limitations
- IMU magnetometer
- Limited sensors, integration error buildup



## Future Work



## Design Directory

[Schematic](Design/Schematic)

[Parts](Design/PartsList)

[Third Party Library Information](Design/ThirdPartyLibInfo)