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

#ifndef VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
#define VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vesc_interface/vesc_interface.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <std_msgs/msg/float64.hpp>


namespace vesc_interface
{
using VescInterfacePtr = std::unique_ptr<vesc_interface::VescInterface>;

class VESC_INTERFACE_PUBLIC VescInterfaceNode : public rclcpp::Node
{
public:
  explicit VescInterfaceNode(const rclcpp::NodeOptions & options);

private:
  VescInterfacePtr vesc_interface_{nullptr};
  void foo();

  // Create publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vesc_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vesc_servo_position_pub_;

  // Create subscribers
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_command_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr
    gear_command_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr
    emergency_command_sub_;

  // Create callbacks
  void control_command_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
  void gear_command_callback(
    const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg);
  void emergency_command_callback(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg);

  float motor_wheel_ratio_;
  float wheel_diameter_;
  float motor_max_rpm_;
  float max_steer_angle_;
};
}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
