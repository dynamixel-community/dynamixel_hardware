// Copyright 2026 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <string>

#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include "dynamixel_hardware/compat.hpp"

namespace
{

constexpr char kDummySystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="use_dummy">true</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint2">
      <param name="id">2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

constexpr char kRealSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="port_name">/dev/ttyUSB0</param>
      <param name="baud_rate">3000000</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint2">
      <param name="id">2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

std::string make_urdf(const std::string & ros2_control_snippet)
{
  return std::string(ros2_control_test_assets::urdf_head) + ros2_control_snippet +
         std::string(ros2_control_test_assets::urdf_tail);
}

// Copied TestableResourceManager pattern (the upstream test header is not
// installed); the constructor signature differs between humble and jazzy+.
class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
#if DXL_HAS_RM_PARAMS_CTOR
  TestableResourceManager(rclcpp::Node & node, const std::string & urdf)
  : hardware_interface::ResourceManager(
      urdf, node.get_node_clock_interface(), node.get_node_logging_interface())
  {
  }
#else
  explicit TestableResourceManager(const std::string & urdf)
  : hardware_interface::ResourceManager(urdf)
  {
  }
#endif
};

void expect_loadable(const std::string & urdf)
{
#if DXL_HAS_RM_PARAMS_CTOR
  rclcpp::Node node("test_load_dynamixel_hardware");
  auto load = [&]() {
      TestableResourceManager rm(node, urdf);
      EXPECT_EQ(1u, rm.system_components_size());
    };
#else
  auto load = [&]() {
      TestableResourceManager rm(urdf);
      EXPECT_EQ(1u, rm.system_components_size());
    };
#endif
  ASSERT_NO_THROW(load());
}

// The plugin must load and initialize through the ResourceManager without
// hardware attached: in dummy mode ...
TEST(TestLoadDynamixelHardware, load_with_use_dummy)
{
  expect_loadable(make_urdf(kDummySystem));
}

// ... and with real serial parameters. on_init only parses parameters; all
// I/O is deferred to on_configure (lifecycle normalization).
TEST(TestLoadDynamixelHardware, load_with_real_params_and_no_hardware)
{
  expect_loadable(make_urdf(kRealSystem));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
