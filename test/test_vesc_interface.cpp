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

#include "gtest/gtest.h"
#include "vesc_interface/vesc_interface.hpp"

TEST(TestVescInterface, GetSpeedTest) {
  std::unique_ptr<vesc_interface::VescInterface> vesc_interface_ =
    std::make_unique<vesc_interface::VescInterface>((float)0.10, (float)0.0942, (float)0.52, (float)0.15, (float)0.85, (float)50000.0);

  float zero_speed = 0.0;
  auto result = vesc_interface_->get_speed(zero_speed);

  float expected_zero_speed = 0.0;
  EXPECT_EQ(result, expected_zero_speed);
}