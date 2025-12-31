// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto urdf_head =
  R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="test_robot">
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="100" velocity="1" lower="-3.14" upper="3.14"/>
  </joint>
  <link name="link2"/>
)";

const auto urdf_tail =
  R"(</robot>
)";

std::string get_urdf_with_hardware(const std::string & hardware_xml)
{
  return urdf_head + hardware_xml + urdf_tail;
}

const auto dynamixel_hardware_xml =
  R"(
  <ros2_control name="DynamixelHardwareSystem" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="use_dummy">true</param>
      <param name="usb_port">/dev/ttyUSB0</param>
      <param name="baud_rate">57600</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

const auto dynamixel_hardware_multi_joint_xml =
  R"(
  <ros2_control name="DynamixelHardwareSystem" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="use_dummy">true</param>
      <param name="usb_port">/dev/ttyUSB0</param>
      <param name="baud_rate">57600</param>
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

}  // namespace

class TestDynamixelHardware : public ::testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(TestDynamixelHardware, load_dynamixel_hardware_plugin)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  ASSERT_TRUE(loader.isClassAvailable("dynamixel_hardware/DynamixelHardware"));
}

TEST_F(TestDynamixelHardware, resource_manager_load_urdf_dummy_mode)
{
  const std::string urdf = get_urdf_with_hardware(dynamixel_hardware_xml);

  hardware_interface::ResourceManager rm(urdf);

  EXPECT_EQ(1u, rm.system_components_size());
}

TEST_F(TestDynamixelHardware, resource_manager_load_urdf_multi_joint_dummy_mode)
{
  const std::string urdf = get_urdf_with_hardware(dynamixel_hardware_multi_joint_xml);

  hardware_interface::ResourceManager rm(urdf);

  EXPECT_EQ(1u, rm.system_components_size());
}

TEST_F(TestDynamixelHardware, configure_activate_deactivate_dummy_mode)
{
  const std::string urdf = get_urdf_with_hardware(dynamixel_hardware_xml);

  hardware_interface::ResourceManager rm(urdf);

  rclcpp_lifecycle::State state_configured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);

  rm.set_component_state("DynamixelHardwareSystem", state_configured);

  auto status_map = rm.get_components_status();
  ASSERT_TRUE(status_map.find("DynamixelHardwareSystem") != status_map.end());
  EXPECT_EQ(
    status_map["DynamixelHardwareSystem"].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  rclcpp_lifecycle::State state_active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  rm.set_component_state("DynamixelHardwareSystem", state_active);

  status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["DynamixelHardwareSystem"].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp_lifecycle::State state_deactivated(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);

  rm.set_component_state("DynamixelHardwareSystem", state_deactivated);

  status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["DynamixelHardwareSystem"].state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(TestDynamixelHardware, read_write_interfaces_dummy_mode)
{
  const std::string urdf = get_urdf_with_hardware(dynamixel_hardware_xml);

  hardware_interface::ResourceManager rm(urdf);

  rclcpp_lifecycle::State state_active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);

  rm.set_component_state("DynamixelHardwareSystem", state_active);

  auto state_interfaces = rm.state_interface_keys();
  EXPECT_EQ(3u, state_interfaces.size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/effort"));

  auto command_interfaces = rm.command_interface_keys();
  EXPECT_EQ(2u, command_interfaces.size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint1/velocity"));

  rm.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));
  rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
