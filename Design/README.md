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
| Autonomous | A Button | Toggles PID car control |
| Autonomous | dPad Up | Increments current PID's current gain |
| Autonomous | dPad Down | Decrements current PID's current gain |
| Autonomous | B Button | Toggle between PID controllers to adjust |
| Autonomous | X Button | Toggle between gains to adjust |