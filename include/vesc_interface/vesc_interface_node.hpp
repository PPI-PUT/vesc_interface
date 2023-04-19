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
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <vesc_msgs/msg/vesc_imu_stamped.hpp>

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

  // publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vesc_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vesc_servo_position_pub_;

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_report_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr
    actuation_status_pub_;

  // subscribers
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_command_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr
    gear_command_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr
    emergency_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    vesc_servo_pos_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr
    vesc_state_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr
    vesc_imu_sub_;

  // callbacks
  void control_command_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
  void gear_command_callback(
    const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg);
  void emergency_command_callback(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg);
  void vesc_servo_pos_callback(
    const std_msgs::msg::Float64::SharedPtr msg);
  void vesc_state_callback(
    const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  void vesc_imu_callback(
    const vesc_msgs::msg::VescImuStamped::SharedPtr msg);

  void publish_raports();

  float motor_wheel_ratio_param_;
  float wheel_diameter_param_;
  float motor_max_rpm_param_;
  float max_steer_angle_param_;

  float servo_max_param_;
  float servo_min_param_;
};
}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
