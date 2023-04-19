// Copyright 2023 Perception for Physical Interaction Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VESC_INTERFACE__VESC_INTERFACE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_HPP_

#include <cstdint>

#include "vesc_interface/visibility_control.hpp"


namespace vesc_interface
{

enum Gear
{
  FORWARD = 2,
  REVERSE = 20,
  PARK = 22
};

struct VelocityModel {
  double lateral_velocity;
  double longitudinal_velocity;
  double heading_rate;
};

struct ActuationStatus {
  double accel_cmd;
  double brake_cmd;
  double steer_cmd;
};

class VESC_INTERFACE_PUBLIC VescInterface
{
public:
  VescInterface() = default;
  VescInterface(float wheel_diameter, float motor_ratio, float max_steer_angle, float servo_min, float servo_max, float motor_max_rpm);
  double get_speed(float & speed_val);
  double get_stearing_angle(float & stearing_val);
  VelocityModel get_velocity_model(double & speed_val);
  ActuationStatus get_actuation_status();

  void set_current_gear(Gear gear);
  void set_emergency_stop(bool & emergency_stop);
  void set_current_steer_angle(double & steer_angle);
  void set_current_heading_rate(double & heading_rate);
  void set_actuation_status_accel(double & accel_cmd);
  void set_actuation_status_steer(double & steer_cmd);

  Gear get_current_gear();
  double get_current_steer_angle();

  double servo_pos_to_steer_angle(double & servo_pos);

private:
  float wheel_diameter_{0.1};
  float motor_ratio_{0.01};
  float max_steer_angle_{0.5};
  float servo_min_{0.15}; // value from vesc_driver config file
  float servo_max_{0.85};
  float motor_max_rpm_{50000.0}; //TOdO: get from config file
 
  Gear current_gear_{Gear::PARK};
  bool emergency_stop_{false};

  // keep values from autoware and return it
  ActuationStatus actuation_status_{0.0, 0.0, 0.0};

  double current_steer_angle_{0.0};
  double current_heading_rate_{0.0};

  double linear_map(float x, float in_min, float in_max, float out_min, float out_max);
  double steer_angle_to_servo_pos(float & steer_angle);
};

}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
