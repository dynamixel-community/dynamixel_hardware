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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dynamixel_hardware/dummy_driver.hpp"
#include "dynamixel_hardware/workbench_driver.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};

#if DXL_HAS_PARAMS_ON_INIT
CallbackReturn DynamixelHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return init_impl(info_);
}
#else
CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return init_impl(info_);
}
#endif

CallbackReturn DynamixelHardware::init_impl(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(logger(), "on_init");

  joints_.resize(info.joints.size(), Joint());
  joint_ids_.resize(info.joints.size(), 0);

  for (size_t i = 0; i < info.joints.size(); i++) {
    const auto & joint = info.joints[i];
    const auto id_it = joint.parameters.find("id");
    if (id_it == joint.parameters.end()) {
      RCLCPP_ERROR(logger(), "Joint '%s' has no 'id' parameter", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    try {
      joint_ids_[i] = static_cast<uint8_t>(std::stoi(id_it->second));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid 'id' parameter '%s': %s", joint.name.c_str(),
        id_it->second.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    RCLCPP_INFO(logger(), "joint '%s': id %d", joint.name.c_str(), joint_ids_[i]);
  }

  const auto & params = info.hardware_parameters;
  use_dummy_ = params.find("use_dummy") != params.end() && params.at("use_dummy") == "true";

  const auto port_name_it = params.find("port_name");
  const auto usb_port_it = params.find("usb_port");
  if (port_name_it != params.end()) {
    port_name_ = port_name_it->second;
  } else if (usb_port_it != params.end()) {
    port_name_ = usb_port_it->second;
    RCLCPP_WARN(
      logger(),
      "The 'usb_port' hardware parameter is deprecated; rename it to 'port_name' in your URDF");
  } else if (!use_dummy_) {
    RCLCPP_ERROR(logger(), "Neither 'port_name' nor 'usb_port' hardware parameter is set");
    return CallbackReturn::ERROR;
  }

  const auto baud_rate_it = params.find("baud_rate");
  if (baud_rate_it != params.end()) {
    try {
      baud_rate_ = std::stoi(baud_rate_it->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger(), "Invalid 'baud_rate' hardware parameter '%s': %s",
        baud_rate_it->second.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  } else if (!use_dummy_) {
    RCLCPP_ERROR(logger(), "The 'baud_rate' hardware parameter is not set");
    return CallbackReturn::ERROR;
  }

  // An injected driver (set_driver_for_testing before on_init) must survive:
  // M3/M4 test fixtures rely on init_impl only creating a driver when none is set.
  if (!driver_) {
    if (use_dummy_) {
      RCLCPP_INFO(logger(), "dummy mode");
      driver_ = std::make_unique<DummyDriver>();
    } else {
      RCLCPP_INFO(logger(), "port_name: %s, baud_rate: %d", port_name_.c_str(), baud_rate_);
      driver_ = std::make_unique<WorkbenchDriver>();
    }
  }

  return CallbackReturn::SUCCESS;
}

#if DXL_HAS_ON_EXPORT
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
DynamixelHardware::on_export_state_interfaces()
{
  RCLCPP_DEBUG(logger(), "on_export_state_interfaces");
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      std::make_shared<hardware_interface::StateInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
DynamixelHardware::on_export_command_interfaces()
{
  RCLCPP_DEBUG(logger(), "on_export_command_interfaces");
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      std::make_shared<hardware_interface::CommandInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(
      std::make_shared<hardware_interface::CommandInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}
#else
std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(logger(), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(logger(), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}
#endif

CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_configure");

  if (!driver_->connect(port_name_, baud_rate_)) {
    RCLCPP_ERROR(
      logger(), "Failed to open '%s': %s", port_name_.c_str(), driver_->last_error().c_str());
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_ids_.size(); i++) {
    if (!driver_->ping(joint_ids_[i])) {
      RCLCPP_ERROR(
        logger(), "Failed to ping id %d: %s", joint_ids_[i], driver_->last_error().c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (!driver_->setup(joint_ids_)) {
    RCLCPP_ERROR(logger(), "Failed to set up handlers: %s", driver_->last_error().c_str());
    return CallbackReturn::ERROR;
  }

  if (set_control_mode(ControlMode::Position, true) != return_type::OK) {
    return CallbackReturn::ERROR;
  }
  if (set_joint_params() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_activate");
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  if (enable_torque(true) != return_type::OK) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_deactivate");
  if (enable_torque(false) != return_type::OK) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_cleanup");
  if (driver_) {
    driver_->disconnect();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_shutdown");
  if (driver_) {
    driver_->disconnect();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_error(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_ERROR(logger(), "on_error: disabling torque and disconnecting (best effort)");
  if (driver_) {
    for (size_t i = 0; i < joint_ids_.size(); i++) {
      driver_->set_torque(joint_ids_[i], false);  // best effort; result ignored
    }
    torque_enabled_ = false;
    driver_->disconnect();
  }
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  if (!driver_->read_states(joint_ids_, positions, velocities, efforts)) {
    // Legacy behavior: log and keep the last known state. Consecutive-failure
    // tolerance and return_type::ERROR escalation land in the params +
    // robustness PR.
    RCLCPP_ERROR(logger(), "%s", driver_->last_error().c_str());
    return return_type::OK;
  }
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].state.position = positions[i];
    joints_[i].state.velocity = velocities[i];
    joints_[i].state.effort = efforts[i];
  }
  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  driver_->tick(period.seconds());

  // Legacy heuristic mode switching, kept verbatim from the pre-refactor
  // implementation: a changed velocity command switches every joint to
  // velocity control, else a changed position command switches every joint
  // to position control, else the current mode's commands are re-sent.

  // Velocity control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.velocity != j.prev_command.velocity;
      }))
  {
    set_control_mode(ControlMode::Velocity);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }

  // Position control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.position != j.prev_command.position;
      }))
  {
    set_control_mode(ControlMode::Position);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {return j.command.effort != 0.0;}))
  {
    RCLCPP_ERROR(logger(), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // If all command values are unchanged, then remain in existing control mode
  // and set corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
    default:  // effort, etc
      RCLCPP_ERROR(logger(), "Control mode not implemented");
      return return_type::ERROR;
  }
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  if (enabled && !torque_enabled_) {
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
      if (!driver_->set_torque(joint_ids_[i], true)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(logger(), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
      if (!driver_->set_torque(joint_ids_[i], false)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(logger(), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (size_t i = 0; i < joint_ids_.size(); ++i) {
      if (!driver_->set_control_mode(joint_ids_[i], ControlMode::Velocity)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(logger(), "Velocity control");
    if (control_mode_ != ControlMode::Velocity) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (size_t i = 0; i < joint_ids_.size(); ++i) {
      if (!driver_->set_control_mode(joint_ids_[i], ControlMode::Position)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(logger(), "Position control");
    if (control_mode_ != ControlMode::Position) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(logger(), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

return_type DynamixelHardware::set_joint_positions()
{
  std::vector<double> commands(joints_.size(), 0.0);
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    commands[i] = joints_[i].command.position;
  }
  if (!driver_->write_positions(joint_ids_, commands)) {
    RCLCPP_ERROR(logger(), "%s", driver_->last_error().c_str());
  }
  return return_type::OK;
}

return_type DynamixelHardware::set_joint_velocities()
{
  std::vector<double> commands(joints_.size(), 0.0);
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    commands[i] = joints_[i].command.velocity;
  }
  if (!driver_->write_velocities(joint_ids_, commands)) {
    RCLCPP_ERROR(logger(), "%s", driver_->last_error().c_str());
  }
  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        int value = 0;
        try {
          value = std::stoi(info_.joints[i].parameters.at(paramName));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            logger(), "Joint '%s' has an invalid '%s' parameter: %s",
            info_.joints[i].name.c_str(), paramName, e.what());
          return CallbackReturn::ERROR;
        }
        if (!driver_->write_item(joint_ids_[i], paramName, value)) {
          RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
          logger(), "%s set to %d for joint '%s'", paramName, value,
          info_.joints[i].name.c_str());
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger DynamixelHardware::logger() const
{
#if DXL_HAS_COMPONENT_LOGGER
  return get_logger();
#else
  return rclcpp::get_logger("DynamixelHardware");
#endif
}

void DynamixelHardware::set_driver_for_testing(std::unique_ptr<DynamixelDriver> driver)
{
  driver_ = std::move(driver);
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
