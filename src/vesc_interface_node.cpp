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

#include "vesc_interface/vesc_interface_node.hpp"

namespace vesc_interface
{

VescInterfaceNode::VescInterfaceNode(const rclcpp::NodeOptions & options)
:  Node("vesc_interface", options)
{
  declare_parameter("motor_wheel_ratio", 0.1);
  declare_parameter("wheel_diameter", 0.1);
  declare_parameter("motor_max_rpm", 0.0);
  declare_parameter("max_steer_angle", 0.4);
  declare_parameter("servo_max", 0.7);
  declare_parameter("servo_min", 0.3);

  get_parameter("motor_wheel_ratio", motor_wheel_ratio_);
  get_parameter("wheel_diameter", wheel_diameter_);
  get_parameter("motor_max_rpm", motor_max_rpm_);
  get_parameter("max_steer_angle", max_steer_angle_);
  get_parameter("servo_max", servo_max_);
  get_parameter("servo_min", servo_min_);


  RCLCPP_ERROR(get_logger(), "servo_max: %f", servo_max_);

  vesc_interface_ = std::make_unique<vesc_interface::VescInterface>(
    wheel_diameter_,
    motor_wheel_ratio_,
    max_steer_angle_, servo_min_,
    servo_max_);
  vesc_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
  vesc_servo_position_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/commands/servo/position", 10);

  control_command_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 10,
    std::bind(&VescInterfaceNode::control_command_callback, this, std::placeholders::_1));

  gear_command_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 10,
    std::bind(&VescInterfaceNode::gear_command_callback, this, std::placeholders::_1));

  emergency_command_sub_ =
    this->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 10,
    std::bind(&VescInterfaceNode::emergency_command_callback, this, std::placeholders::_1));
}

void VescInterfaceNode::control_command_callback(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  double speed = vesc_interface_->get_speed(msg->longitudinal.speed);
  double stearing_angle = vesc_interface_->get_stearing_angle(msg->lateral.steering_tire_angle);

  auto speed_msg = std_msgs::msg::Float64();
  speed_msg.data = speed;

  auto stearing_angle_msg = std_msgs::msg::Float64();
  stearing_angle_msg.data = stearing_angle;

  vesc_speed_pub_->publish(speed_msg);
  vesc_servo_position_pub_->publish(stearing_angle_msg);
}

void VescInterfaceNode::gear_command_callback(
  const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
  vesc_interface_->set_current_gear((Gear)msg->command);
}

void VescInterfaceNode::emergency_command_callback(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg)
{
  vesc_interface_->set_emergency_stop(msg->emergency);
}

}  // namespace vesc_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_interface::VescInterfaceNode)
