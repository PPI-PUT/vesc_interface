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

namespace vesc_interface
{

VescInterface::VescInterface()
{
}

void VescInterface::setParameters(int64_t param_name)
{
  param_name_ = param_name;
}

int64_t VescInterface::printHello() const
{
  std::cout << "Hello World, " << param_name_ << std::endl;
  return param_name_;
}

}  // namespace vesc_interface
