# rc-car

## Car Controls

| State | Input | Behavior |
|------:|------:|:---------|
| Any | Left Bumper | Toggles Armed/Disarmed |
|   |   |   |
| Disarmed | dPad Down + A Button | Performs a Complete IMU Calibration |
| Disarmed | dPad Up + A Button | Performs a Local Bias Only IMU Calibration |
|   |   |   |
| Manual | Left Stick | Drives the Vehicle Forward/Backward |
| Manual | Right Stick  | Turns the Vehicle |
| Manual | X Button | Toggles Motor Throttle |
|   |   |   |
| Manual/Autonomous | Y Button | Toggles Manual/Autonomous |
|   |   |   |
| Autonomous | A Button | Toggles PID Control |
| Autonomous | dPad Right | Increments Kp |
| Autonomous | dPad Left | Decrements Kp |
| Autonomous | dPad Up | Increments Ki |
| Autonomous | dPad Down | Decrements Ki |
| Autonomous | B Button | Increments Kd |
| Autonomous | X Button | Decrements Kd |