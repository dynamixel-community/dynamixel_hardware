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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr int kDefaultBaudRate = 57600;

return_type DynamixelHardware::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());

  // for (const hardware_interface::ComponentInfo & joint : info_.joints) {
  // }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].position));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command));
  }

  return command_interfaces;
}

return_type DynamixelHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Starting ...please wait...");

  for (uint i = 0; i < joints_.size(); i++) {
    if (std::isnan(joints_[i].position)) {
      joints_[i].position = 0.0;
      joints_[i].command = 0.0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "System Sucessfully started!");

  return return_type::OK;
}

return_type DynamixelHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Stopping ...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "System sucessfully stopped!");

  return return_type::OK;
}

hardware_interface::return_type DynamixelHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Reading...");

  for (uint i = 0; i < joints_.size(); i++) {
  }
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Joints sucessfully read!");

  return return_type::OK;
}

hardware_interface::return_type dynamixel_hardware::DynamixelHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Writing...");

  for (uint i = 0; i < joints_.size(); i++) {
  }
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Joints sucessfully written!");

  return return_type::OK;
}
}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
