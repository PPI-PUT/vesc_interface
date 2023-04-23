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

  get_parameter("motor_wheel_ratio", motor_wheel_ratio_param_);
  get_parameter("wheel_diameter", wheel_diameter_param_);
  get_parameter("motor_max_rpm", motor_max_rpm_param_);
  get_parameter("max_steer_angle", max_steer_angle_param_);
  get_parameter("servo_max", servo_max_param_);
  get_parameter("servo_min", servo_min_param_);

  vesc_interface_ = std::make_unique<vesc_interface::VescInterface>(
    wheel_diameter_param_,
    motor_wheel_ratio_param_,
    max_steer_angle_param_, servo_min_param_,
    servo_max_param_, motor_max_rpm_param_);

  vesc_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/commands/motor/speed", 10);

  vesc_servo_position_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/commands/servo/position", 10);

  gear_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 1);

  steering_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 1);

  control_mode_report_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", 1);

  velocity_report_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1);

  actuation_status_pub_ =
    this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status", 1);

  rclcpp::QoS qos_profile_ackermann = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(1)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .avoid_ros_namespace_conventions(false);

  control_command_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", qos_profile_ackermann,
    std::bind(&VescInterfaceNode::control_command_callback, this, std::placeholders::_1));

  rclcpp::QoS qos_profile_gearcommand = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(1)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .avoid_ros_namespace_conventions(false);

  gear_command_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", qos_profile_gearcommand,
    std::bind(&VescInterfaceNode::gear_command_callback, this, std::placeholders::_1));

  emergency_command_sub_ =
    this->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 10,
    std::bind(&VescInterfaceNode::emergency_command_callback, this, std::placeholders::_1));

  vesc_servo_pos_sub_ =
    this->create_subscription<std_msgs::msg::Float64>(
    "/sensors/servo_position_command", 10,
    std::bind(&VescInterfaceNode::vesc_servo_pos_callback, this, std::placeholders::_1));

  vesc_state_sub_ =
    this->create_subscription<vesc_msgs::msg::VescStateStamped>(
    "/sensors/core", 10,
    std::bind(&VescInterfaceNode::vesc_state_callback, this, std::placeholders::_1));

  vesc_imu_sub_ =
    this->create_subscription<vesc_msgs::msg::VescImuStamped>(
    "/sensors/imu", 10,
    std::bind(&VescInterfaceNode::vesc_imu_callback, this, std::placeholders::_1));
}

void VescInterfaceNode::control_command_callback(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  double ackermann_vel = msg->longitudinal.speed;
  double ackermann_steer = msg->lateral.steering_tire_angle;

  double speed = vesc_interface_->get_speed(ackermann_vel);
  double stearing_angle = vesc_interface_->get_stearing_angle(ackermann_steer);

  auto speed_msg = std_msgs::msg::Float64();
  speed_msg.data = speed;

  auto stearing_angle_msg = std_msgs::msg::Float64();
  stearing_angle_msg.data = stearing_angle;

  vesc_speed_pub_->publish(speed_msg);
  vesc_servo_position_pub_->publish(stearing_angle_msg);

  vesc_interface_->set_actuation_status_steer(ackermann_steer);
  vesc_interface_->set_actuation_status_accel(ackermann_vel);
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

void VescInterfaceNode::vesc_servo_pos_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  double steer_angle = vesc_interface_->servo_pos_to_steer_angle(msg->data);
  vesc_interface_->set_current_steer_angle(steer_angle);
}

void VescInterfaceNode::vesc_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  auto vel_model = vesc_interface_->get_velocity_model(msg->state.speed);

  auto velocity_report_msg = autoware_auto_vehicle_msgs::msg::VelocityReport();
  velocity_report_msg.header.stamp = this->now();
  velocity_report_msg.header.frame_id = "base_link";
  velocity_report_msg.longitudinal_velocity = vel_model.longitudinal_velocity;
  velocity_report_msg.lateral_velocity = vel_model.lateral_velocity;
  velocity_report_msg.heading_rate = vel_model.heading_rate;

  velocity_report_pub_->publish(velocity_report_msg);

  publish_raports();
}

void VescInterfaceNode::vesc_imu_callback(const vesc_msgs::msg::VescImuStamped::SharedPtr msg)
{
  vesc_interface_->set_current_heading_rate(msg->imu.angular_velocity.z);
}

void VescInterfaceNode::publish_raports()
{
  auto control_mode_report_msg = autoware_auto_vehicle_msgs::msg::ControlModeReport();
  control_mode_report_msg.stamp = this->now();
  control_mode_report_msg.mode = control_mode_report_msg.AUTONOMOUS;
  control_mode_report_pub_->publish(control_mode_report_msg);

  ActuationStatus actuation_status = vesc_interface_->get_actuation_status();
  auto actuation_status_report_msg = tier4_vehicle_msgs::msg::ActuationStatusStamped();
  actuation_status_report_msg.header.stamp = this->now();
  actuation_status_report_msg.header.frame_id = "base_link";
  actuation_status_report_msg.status.accel_status = actuation_status.accel_cmd;
  actuation_status_report_msg.status.brake_status = actuation_status.brake_cmd;
  actuation_status_report_msg.status.steer_status = actuation_status.steer_cmd;
  actuation_status_pub_->publish(actuation_status_report_msg);

  auto steering_report_msg = autoware_auto_vehicle_msgs::msg::SteeringReport();
  steering_report_msg.stamp = this->now();
  steering_report_msg.steering_tire_angle = vesc_interface_->get_current_steer_angle();
  steering_report_pub_->publish(steering_report_msg);

  auto gear_report_msg = autoware_auto_vehicle_msgs::msg::GearReport();
  gear_report_msg.stamp = this->now();
  gear_report_msg.report = vesc_interface_->get_current_gear();
  gear_report_pub_->publish(gear_report_msg);
}

}  // namespace vesc_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_interface::VescInterfaceNode)
