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

class VESC_INTERFACE_PUBLIC VescInterface
{
public:
  VescInterface(float wheel_diameter, float motor_ratio, float max_steer_angle);
  double get_speed(float & speed_val);
  double get_stearing_angle(float & stearing_val);

  void set_current_gear(Gear gear);
  void set_emergency_stop(bool & emergency_stop);

private:
  float wheel_diameter_{0.0};
  float motor_ratio_{0.0};
  float max_steer_angle_{0.0};
  float servo_min_{0.0}; //value from vesc_driver config file
  float servo_max_{1.0};
 
  Gear current_gear_{Gear::PARK};
  bool emergency_stop_{false};

  double linear_map(float x, float in_min, float in_max, float out_min, float out_max);
};

  

}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
