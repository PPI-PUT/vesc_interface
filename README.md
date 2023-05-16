# vesc_interface
<!-- Required -->
<!-- Package description -->
Package provides an interface to communicate the VESC motor controller with Autoware stack. It is implementation of vehicle interface component of Autoware stack for VESC motor driver.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to vesc_interface
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch vesc_interface vesc_interface.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/control/command/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | autoware steering command |
| `/control/command/emergency_cmd` | tier4_vehicle_msgs::msg::VehicleEmergencyStamped | autoware emergency command |
| `/control/command/gear_cmd` | autoware_auto_vehicle_msgs::msg::GearCommand | autoware gear command |
| `/sensors/servo_position_command` | std_msgs::msg::Float64 | Vesc servo position |
| `/sensors/core` | vesc_msgs::msg::VescStateStamped | Vesc State information, including current motor speed |
| `/sensors/imu` | vesc_msgs::msg::VescImuStamped | Vesc IMU information |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/commands/motor/speed` | std_msgs::msg::Float64 | Speed message for VESC in rpm |
| `/commands/servo/position` | std_msgs::msg::Float64 | Servo position message for VESC in (0 - 1) range where 0.5 is straight |
| `/vehicle/status/control_mode` | autoware_auto_vehicle_msgs::msg::ControlModeReport | Current contol mode kept by vessc_interface |
| `/vehicle/status/gear_status` | autoware_auto_vehicle_msgs::msg::GearReport | Gear status published as a feedback to Autoware Control component |
| `/vehicle/status/steering_status` | autoware_auto_vehicle_msgs::msg::SteeringReport | Steering status published as a feedback to Autoware Control component |
| `/vehicle/status/velocity_status` | autoware_auto_vehicle_msgs::msg::VelocityReport | Velocity status published as a feedback to Autoware Control component |
| `/vehicle/status/actuation_status` | tier4_vehicle_msgs::msg::ActuationStatusStamped | Actuation status published as a feedback to Autoware Control component |


### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `motor_wheel_ratio` | float  | Ratio from motor to wheel output |
| `wheel_diameter` | float  | Diameter of vehicle wheel in meters |
| `motor_max_rpm` | float  | Maximal value of motor rpm |
| `max_steer_angle` | float  | Maximal steering angle in radians |


## References / External links
<!-- Optional -->
- about Autoware vehicle interface [link](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/)
- about ROS2 VESC motor driver [link](https://github.com/f1tenth/vesc/tree/ros2)