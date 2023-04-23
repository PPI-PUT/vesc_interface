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
  float servo_min, float servo_max, float motor_max_rpm)
: wheel_diameter_param_(wheel_diameter),
  motor_ratio_(motor_ratio),
  max_steer_angle_param_(max_steer_angle),
  servo_min_param_(servo_min),
  servo_max_param_(servo_max),
  motor_max_rpm_param_(motor_max_rpm),
  max_vel_m_s_(motor_max_rpm_param_ * motor_ratio_ * M_PI * wheel_diameter_param_ / 60.0)
{
}

double VescInterface::get_speed(double & speed_val)
{
  if (current_gear_ == Gear::PARK || emergency_stop_) {
    return 0.0;
  }

  auto rpm_speed = speed_val * 60 / (M_PI * wheel_diameter_param_ * motor_ratio_);

  if (current_gear_ == Gear::REVERSE && rpm_speed > 0) {
    return 0.0;
  }
  if (current_gear_ == Gear::FORWARD && rpm_speed < 0) {
    return 0.0;
  }

  return rpm_speed;
}

double VescInterface::get_stearing_angle(double & stearing_val)
{
  if (current_gear_ == emergency_stop_) {
    return 0.0;
  }

  auto clamped_stearing_cal = std::clamp(
    stearing_val, -(double)max_steer_angle_param_,
    (double)max_steer_angle_param_);
  return steer_angle_to_servo_pos(clamped_stearing_cal);
}

VelocityModel VescInterface::get_velocity_model(double & speed_val)
{
  VelocityModel velocity_model;
  auto m_s_speed = speed_val * motor_ratio_ * M_PI * wheel_diameter_param_ / 60.0;

  velocity_model.longitudinal_velocity = m_s_speed * std::cos(this->current_steer_angle_);
  velocity_model.lateral_velocity = m_s_speed * std::sin(this->current_steer_angle_);
  if (this->current_steer_angle_ < 0) {
    velocity_model.lateral_velocity *= -1;
  }
  velocity_model.heading_rate = this->current_heading_rate_;

  return velocity_model;
}

ActuationStatus VescInterface::get_actuation_status()
{
  return this->actuation_status_;
}

void VescInterface::set_current_gear(Gear gear)
{
  current_gear_ = gear;
}

void VescInterface::set_emergency_stop(bool & emergency_stop)
{
  emergency_stop_ = emergency_stop;
}

void VescInterface::set_current_steer_angle(double & steer_angle)
{
  current_steer_angle_ = steer_angle;
}

void VescInterface::set_current_heading_rate(double & heading_rate)
{
  current_heading_rate_ = heading_rate;
}

void VescInterface::set_actuation_status_accel(double & accel)
{
  this->actuation_status_.accel_cmd = linear_map(accel, 0.0, max_vel_m_s_, 0.0, 1.0);
}

void VescInterface::set_actuation_status_steer(double & steer)
{
  this->actuation_status_.steer_cmd = linear_map(
    steer, -max_steer_angle_param_,
    max_steer_angle_param_, -1, 1);
}

Gear VescInterface::get_current_gear()
{
  return current_gear_;
}

double VescInterface::get_current_steer_angle()
{
  return current_steer_angle_;
}

double VescInterface::linear_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double VescInterface::servo_pos_to_steer_angle(double & servo_pos)
{
  return linear_map(
    servo_pos, servo_min_param_, servo_max_param_, -max_steer_angle_param_,
    max_steer_angle_param_);
}

double VescInterface::steer_angle_to_servo_pos(double & steer_angle)
{
  return linear_map(
    steer_angle, -max_steer_angle_param_, max_steer_angle_param_, servo_min_param_,
    servo_max_param_);
}

}  // namespace vesc_interface
