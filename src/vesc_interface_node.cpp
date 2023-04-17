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
  vesc_interface_ = std::make_unique<vesc_interface::VescInterface>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  vesc_interface_->setParameters(param_name);
  this->foo();
}

void VescInterfaceNode::foo()
{
  vesc_interface_->printHello();
}

}  // namespace vesc_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_interface::VescInterfaceNode)
