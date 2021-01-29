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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <map>
#include <vector>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::return_type;

namespace dynamixel_hardware
{
struct Joint
{
  std::string name;
  double command{0.0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

class DynamixelHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type start() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type stop() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write() override;

private:
  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
