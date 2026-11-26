## Features / Overview
- Extended Kalman Filter (EKF) estimates pose in real time by fusing IMU and odometry measurements
- Autonomous obstacle-free return to home by dead reckoning navigation capability using EKF state estimation and PID controller in a Sense -> Estimate -> Control cycle 
- State machine architecture that safely handles faults, state transitions, and hardware fail-safes on communication loss
- UDP telemetry pipeline for remote logging with pre-allocated queuing and a dedicated FreeRTOS task to separate logging from the time-constrained, prioritized control loop 
- A fault-tolerant embedded control system in C++ on a resource-constrained ESP32 microcontroller with FreeRTOS priority-scheduled tasks, a fixed rate control loop, and hardware abstraction across motors, sensors, and vehicle classes
- Remote control manual operation via Bluetooth
- Python tools that parse logs and plot EKF and PID performance across runs on a shared time axis

<p align="center">
  <img src="./Design/Images/DAR-C/IMG_2332.jpeg" width="32%"/>
  <img src="./Design/Images/DAR-C/IMG_2328.jpeg" width="32%"/>
  <img src="./Design/Images/DAR-C/IMG_2334.jpeg" width="32%"/>
</p>

https://github.com/user-attachments/assets/d5b72bf7-f268-4b66-aeb9-6d17970775e9

https://github.com/user-attachments/assets/d52f3077-a2a9-420c-96de-9e00e28c038f

https://github.com/user-attachments/assets/f118bfb4-4f4a-4d74-a1b2-34342b7d11e6


## Skills developed
- Kalman Filtering, Extended Kalman Filtering
- PID control
- Embedded real-time control software, RTOS, prioritized control tasks
- UDP, remote telemetry logging
- State machine design
- Data processing and plotting (Python, Pandas, Matplotlib) 
- Circuitry, circuit schematic design, 3D modeling



## Design Considerations
### Extended Kalman Filter
The Kalman Filter fuses acceleration and gyro data from an IMU and wheel encoder data from the motors to estimate the car's state. The state vector $\bar{x}$ evolves as a function of the previous state: 

```math
\begin{array}{c}
\bar{x} = [x,\ y,\ v,\ a,\ \theta,\ \omega]^T \\[6pt]
\bar{x}_{k+1} = f(\bar{x}_k) 
\end{array}
```

Since this motion model is nonlinear, an Extended Kalman Filter (EKF) was required. The Jacobian below was used to linearize the system at the current state estimate

```math
\begin{array}{rcl}
x_{k+1} & = & x_k + (v_k\,\Delta t + a_k\,\Delta t^2/2)\cos(\theta_k) \\[6pt]
y_{k+1} & = & y_k + (v_k\,\Delta t + a_k\,\Delta t^2/2)\sin(\theta_k) \\[6pt]
v_{k+1} & = & v_k + a_k\,\Delta t \\[6pt]
a_{k+1} & = & a_k \\[6pt]
\theta_{k+1} & = & \theta_k + \omega_k\,\Delta t \\[6pt]
\omega_{k+1} & = & \omega_k
\end{array}
\quad \xrightarrow{\ \ \partial f / \partial \bar{x}\ \ } \quad
A_{jacobian} = \begin{bmatrix}
1 & 0 & \Delta t\cos\theta & \Delta t^2\cos\theta/2 & -\left(v\,\Delta t + a\,\Delta t^2/2\right)\sin\theta & 0 \\[10pt]
0 & 1 & \Delta t\sin\theta & \Delta t^2\sin\theta/2 & \left(v\,\Delta t + a\,\Delta t^2/2\right)\cos\theta & 0 \\[10pt]
0 & 0 & 1 & \Delta t & 0 & 0 \\[6pt]
0 & 0 & 0 & 1 & 0 & 0 \\[6pt]
0 & 0 & 0 & 0 & 1 & \Delta t \\[6pt]
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
```

<br><br>

Measurement update timing, $\Delta t$
- Although a fixed rate control loop was implemented, a jitter analysis was not performed, therefore a variable $\Delta t$ (measured at the time of measurement) was used in the prediction step of the EKF and PID loop to increase accuracy of the filter

Variable motion model and sensor noise matrices 
- The car experienced varying amounts of wheel slippage which affected the filter's motion model, $Q$, and encoder sensor noise, $R$, depending on how aggressively the car was being driven and the terrain it was being driven on. This increased the EKF's accuracy as its trust in the encoder sensors changed accordingly depending on the driving regime.
- Driving intensely -> $Q \uparrow$ (motion noise increases due to wheel slip), $R_{encoders} \uparrow$ (trust encoders less)
- Stationary -> $Q \downarrow$ (motion noise decreases due to no wheel slip), $R_{encoders} \downarrow$ (trust encoders more)
- This change also prevents unbounded drift in the car's heading estimate while stationary. This drift came from integrating the gyro's turn rate measurements which had a bias of $−0.0125\, \text{rad/s}$ that was difficult to remove entirely. This bias was near constant (proved to grow over time very slowly with python analysis), and therefore, when integrated, produced a linearly increasing drift in the car's heading estimate, even when completely stationary. The variable $R$ and $Q$ matrices changed how much the gyro measurement was trusted when the car was completely stationary, preventing the integration drift

Tuning and Verification
- Initial guesses for $R$ were selected by calculating variances of the accelerometer and gyroscope using recorded stationary data 
- $Q$ was tuned empirically starting with educated guesses about terrain and wheel slip
- Normalized innovation, $\nu$, was calculated using the following formula: $\nu=y/\sqrt{S}$, where $y$ is the innovation $y=z-Hx_{predicted}$, and $S$ is the innovation's predicted variance, calculated as $S=HPH^T+R$.
- When tuning, the normalized innovation was centered around 0 (mean = 0), indicating the innovations for turn rate, velocity, and acceleration were unbiased. The standard deviation was aimed to be equal to 1 (the innovation or "surprise" was exactly what the filter predicted it should be based on $P$, $Q$, and $R$).
- On each iteration, the estimated state and diagonal values of $P$ (uncertainties of each state variable) were plotted and compared against a visually confirmed approximate path of travel. Example plots are shown below:

<table>
  <tr>
    <th colspan="2" align="center">Regime-Adaptive EKF Driving in Small Rectangle: States, Normalized Innovation, and Covariance (P) Diagonals </th>
  </tr>
  <tr>
    <td align="center">
      <img src="./Design/Images/Plots/EKF-Small-Rectangle-Variable-Regime-State-and-Normalized-Innovation.png" width="100%" />
      <br/><sub><b>Figure 1.</b> Estimated states and normalized innovation</sub>
    </td>
    <td align="center">
      <img src="./Design/Images/Plots/EKF-Small-Rectangle-Variable-Regime-P-Matrix-Diagonals.png" width="100%" />
      <br/><sub><b>Figure 2.</b> Covariance (P matrix) diagonals</sub>
    </td>
  </tr>
  <tr>
    <th colspan="2" align="center">Single Regime EKF Driving in Small Rectangle With Curved Corners: States, Normalized Innovation, and Covariance (P) Diagonals</th>
  </tr>
  <tr>
    <td align="center">
      <img src="./Design/Images/Plots/EKF-Small-Rectangle-With-Curves-Single-Regime-State-and-Normalized-Innovation.png" width="100%" />
      <br/><sub><b>Figure 3.</b> Estimated states and normalized innovation</sub>
    </td>
    <td align="center">
      <img src="./Design/Images/Plots/EKF-Small-Rectangle-With-Curves-Single-Regime-P-Matrix-Diagonals.png" width="100%"/>
      <br/><sub><b>Figure 4.</b> Covariance (P matrix) diagonals</sub>
    </td>
  </tr>
</table>

### PID Controllers
- The car ran two autonomous controllers, one PI controller for the car's forward velocity and one PID controller for its heading
- The two controllers' outputs were fed to the car's control class concurrently allowing it to move forward and steer simultaneously towards its destination

Velocity PI Controller
- Only proportional and integral gains were used for speed control. Adding a derivative gain to this controller caused large spikes in the step response. This is attributed to noisy accelerometer data feeding velocity estimates and quantized velocity measurements. The stationary accelerometer noise compared to its typical signal is ~16% ($var = 0.027\, (\text{m/s}^2)^2 \rightarrow \sigma = 0.164\, \text{m/s}^2$ compared to a normal signal of $1\, \text{m/s}^2$). Whereas with the stationary gyro, its noise compared to its typical signal is ~0.08% ($var = 8.035852\times10^{-7}\, (\text{rad/s})^2 \rightarrow \sigma = 8.96\times10^{-4}\, \text{rad/s}$ and a normal signal of $1\, \text{rad/s}$). The normal signals used in these calculations came from the values shown in the EKF state estimate plots. This is enough to cause significant jitter in velocity estimate and less jitter in the heading estimate. The EKF state plots are consistent with this. Furthermore, since the wheel encoders are only capable of measuring in steps, and our $\Delta t$ was very small, the velocity estimate could jump up and down and the derivative gain would see accelerations much higher or lower than the true acceleration. 

Heading PID Controller
- A derivative gain was useful in this case because the turn rate estimate was dominated by the IMU gyro (gyro had a much smaller covariance than the encoder in the $R$ matrix), which had small noise compared with the real turn rate, allowing the derivative term to differentiate a smooth error curve, leading to a successful damping effect.

Deadband Compensation in Autonomous Return to Home
- As the heading error became small the PID output wasn't strong enough to overcome friction and the car was unable to close the error completely. This wasn't a big problem when testing the velocity and heading controllers individually, but when combining inputs in the autonomous return to home navigation, the problem became apparent
- To solve this, the heading PID output is thresholded so the steering command is only ever left (-1), right (+1), or none (0), which means the car turns at a fixed rate when the PID demand saturates and drives straight otherwise. This is effectively on/off steering with a deadband around the target heading
- The trade-off is that the controller can no longer approach the target smoothly, so heading accuracy is limited by the deadband and fixed turn rate, but this is acceptable for the purposes of this project

Verification
- Step response data was recorded and plotted for analysis in tuning the PID controllers. Example response plots without the thresholded steering are shown below

<p align="center">
  <img src="./Design/Images/Plots/VelocityPID.png" width="49%"/>
  <img src="./Design/Images/Plots/ThetaPID.png" width="49%"/>
</p>

### Remote Logging
- Singleton design pattern to ensure one logger and one log queue
- Separate logging FreeRTOS task with lower priority than control task to ensure control is not held up by telemetry
- Non-blocking queue, if the queue is full messages are purposely dropped so the control task is not held up 
- Pre-allocated memory for log message queue to prevent dynamic memory allocation faults with queue size being determined by available RAM, expected log rate, and message size
- Log messages sent over UDP for its low overhead as opposed to TCP since a delivery acknowledgment was not necessary and an occasionally dropped log was acceptable
- Formatted consistently with timestamp so python scripts can parse logged data for analysis and tuning 

### State Machine and Fault Tolerance
- Explicit state machine with states: Manual, Autonomous, Bluetooth Pairing, Connected and Disarmed, Sensor Calibration
- State transitions handled explicitly and each mode runs its own per-cycle operations, including on entry setup
- Safety critical arming: a connection or reconnection always lands in Connected and Disarmed, forcing an operator to arm the system once it's connected in order to operate it
- Fault handling such as a sudden disconnect forces Pairing mode and stops the motors
- Operator can switch to Manual at any time during Autonomous control with single button push
- Status LEDs show the mode (slow blink while pairing, fast blink in autonomous, red LED while armed, yellow while calibrating sensors)
- Bounded WiFi connection retries 
 
### Embedded Software Design
- Designed with separate FreeRTOS tasks for logging and control	
	- Fixed memory, no `new`/`malloc` calls
	- Allow task prioritization and allocation of operation time based on task priority (control task higher priority than logging)
	- Fixed timing loop control task for EKF and PID
	- Ability to add future tasks to expand capabilities 
- Hardware abstraction across motors, sensors, and vehicle classes
	- Build abstraction interfaces to make the system hardware agnostic, allowing us to easily interchange future hardware (IMUs, motors, encoders)



## Verification ("Did we build it right?")
- Analysis - tuning Kalman Filter - normalized innovation was calculated and plotted. The mean was verified to be approximately zero, and the standard deviation was verified to be approximately 1
- Analysis - tuning PID controllers - both the speed controller and the heading controllers' step responses were plotted with varying gains and at varying step responses
- Test - removing sensor bias - tested a calibration function that removes sensor bias by confirming that sensor bias was decreased after applying the bias removal function
- Test - coordinate axes - tilted the car up, down, left, and right while logging acceleration and gyroscope data to verify the coordinate axes aligned with the design
- Demonstration - state machine - verified that all state transitions were expected and the car followed the intended state machine design without problems or unexpected behavior 
- Demonstration - fault tolerance - verified that when the controller disconnects, the car stops and enters the pairing state


## Validation ("Did we build the right thing?")
- Measured accelerometer and gyroscope noise while stationary to identify real life operating conditions the car would experience and tuned the Kalman Filter using those values
- Drove the car on a variety of pre-measured and marked routes such as a rectangle, circle, or combination of turns and straight lines. Then confirmed that the Kalman Filter's estimated state aligned with the known path
- Successfully tested the return to home function with multiple different driving conditions, integrated all parts of the system 



## Project Limitations
- The biggest limitation of this project was reduced sensor capabilities. The car operated with an IMU that measured acceleration (g), a gyroscope that measured turn rate (rad/s), and wheel encoders that allowed indirect calculation of velocity (m/s). In addition to the accelerometer, the IMU had a magnetometer that indicated heading, but was not consistently providing accurate data, even with calibration efforts. To estimate heading, velocity, and position, the gyroscope measurements were integrated once, and the accelerometer measurements were integrated twice. This led to unbounded error growth because the car did not have a sensor that directly measured heading or position, which would have decreased uncertainty in the EKF
- Another limitation in EKF tuning was the inability to track the driving path exactly and compare it to the filter's estimated state. To compensate, driving paths were measured and marked ahead of time to produce a known approximate path of travel to compare the filter against



## Design Directory

[Schematic](Design/Schematic)

[Parts](Design/PartsList)

[Third Party Library Information](Design/ThirdPartyLibInfo)
