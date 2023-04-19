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

#include "vesc_interface/vesc_interface.hpp"

#include <iostream>
#include <cmath>
#include <algorithm>

namespace vesc_interface
{

VescInterface::VescInterface(
  float wheel_diameter, float motor_ratio, float max_steer_angle,
  float servo_min, float servo_max)
: wheel_diameter_(wheel_diameter),
  motor_ratio_(motor_ratio),
  max_steer_angle_(max_steer_angle),
  servo_min_(servo_min),
  servo_max_(servo_max)
{
}

double VescInterface::get_speed(float & speed_val)
{
  if (current_gear_ == Gear::PARK || emergency_stop_) {
    return 0.0;
  }

  auto rmp_speed = speed_val * 60 / (2 * M_PI * wheel_diameter_ * motor_ratio_);

  if (current_gear_ == Gear::REVERSE && rmp_speed > 0) {
    return 0.0;
  }
  if (current_gear_ == Gear::FORWARD && rmp_speed < 0) {
    return 0.0;
  }

  return rmp_speed;
}

double VescInterface::get_stearing_angle(float & stearing_val)
{
  if (current_gear_ == emergency_stop_) {
    return 0.0;
  }

  auto clamped_stearing_cal = std::clamp(stearing_val, -max_steer_angle_, max_steer_angle_);
  return linear_map(
    clamped_stearing_cal, -max_steer_angle_, max_steer_angle_, servo_min_,
    servo_max_);
}

void VescInterface::set_current_gear(Gear gear)
{
  current_gear_ = gear;
}

void VescInterface::set_emergency_stop(bool & emergency_stop)
{
  emergency_stop_ = emergency_stop;
}

double VescInterface::linear_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

}  // namespace vesc_interface
