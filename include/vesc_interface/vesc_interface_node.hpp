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
};
}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_NODE_HPP_
