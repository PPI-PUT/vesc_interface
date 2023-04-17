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

class VESC_INTERFACE_PUBLIC VescInterface
{
public:
  VescInterface();
  void setParameters(int64_t param_name);
  int64_t printHello() const;

private:
  int64_t param_name_{123};
};

}  // namespace vesc_interface

#endif  // VESC_INTERFACE__VESC_INTERFACE_HPP_
