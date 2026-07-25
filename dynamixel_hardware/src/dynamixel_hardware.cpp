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
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <unordered_map>
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

  joints_.clear();
  joints_.resize(info.joints.size(), Joint());

  for (size_t i = 0; i < info.joints.size(); i++) {
    const auto & joint_info = info.joints[i];
    const auto & joint_params = joint_info.parameters;
    auto & joint = joints_[i];

    const auto id_it = joint_params.find("id");
    if (id_it == joint_params.end()) {
      RCLCPP_ERROR(logger(), "Joint '%s' has no 'id' parameter", joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
    try {
      joint.id = static_cast<uint8_t>(std::stoi(id_it->second));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid 'id' parameter '%s': %s", joint_info.name.c_str(),
        id_it->second.c_str(), e.what());
      return CallbackReturn::ERROR;
    }

    const auto mode_it = joint_params.find("control_mode");
    if (mode_it != joint_params.end() &&
      !parse_control_mode(mode_it->second, joint.configured_mode))
    {
      RCLCPP_ERROR(
        logger(),
        "Joint '%s' has an unknown 'control_mode' parameter '%s' (expected one of: position, "
        "extended_position, multi_turn, current_based_position, velocity, current, torque, pwm)",
        joint_info.name.c_str(), mode_it->second.c_str());
      return CallbackReturn::ERROR;
    }
    joint.active_mode = joint.configured_mode;

    const auto torque_constant_it = joint_params.find("torque_constant");
    if (torque_constant_it != joint_params.end()) {
      try {
        joint.torque_constant = std::stod(torque_constant_it->second);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          logger(), "Joint '%s' has an invalid 'torque_constant' parameter '%s': %s",
          joint_info.name.c_str(), torque_constant_it->second.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
      if (joint.torque_constant <= 0.0) {
        RCLCPP_ERROR(
          logger(), "Joint '%s' has a non-positive 'torque_constant' parameter '%s' (Nm/A > 0)",
          joint_info.name.c_str(), torque_constant_it->second.c_str());
        return CallbackReturn::ERROR;
      }
    }

    // Zero would make the gear conversion divide by zero; negative ratios are
    // deliberately allowed -- they invert the rotation direction (#95/#94).
    const auto gear_ratio_status =
      parse_double_param(joint_params, "gear_ratio", joint.gear_ratio, false);
    if (gear_ratio_status != CallbackReturn::SUCCESS) {
      return gear_ratio_status;
    }

    joint.state.position = std::numeric_limits<double>::quiet_NaN();
    joint.state.velocity = std::numeric_limits<double>::quiet_NaN();
    joint.state.effort = std::numeric_limits<double>::quiet_NaN();
    joint.command.position = std::numeric_limits<double>::quiet_NaN();
    joint.command.velocity = std::numeric_limits<double>::quiet_NaN();
    joint.command.effort = std::numeric_limits<double>::quiet_NaN();
    joint.command.pwm = 0.0;
    joint.prev_command = joint.command;
    RCLCPP_INFO(
      logger(), "joint '%s': id %d, control_mode %s", joint_info.name.c_str(), joint.id,
      mode_name(joint.configured_mode));
  }

  pending_modes_.assign(joints_.size(), ControlMode::Position);
  pending_switch_.assign(joints_.size(), false);
  pending_legacy_.assign(joints_.size(), false);
  pending_claims_.assign(joints_.size(), std::set<std::string>{});

  const auto & params = info.hardware_parameters;
  use_dummy_ = params.find("use_dummy") != params.end() && params.at("use_dummy") == "true";

  const auto torque_enable_it = params.find("torque_enable");
  torque_enable_param_ = torque_enable_it == params.end() || torque_enable_it->second != "false";

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

  // baud_rate ends up as the uint32_t passed to DynamixelWorkbench::init();
  // 0 or negative would be meaningless (a zero/wrapped bit rate), so >= 1.
  const auto baud_status = parse_int_param(params, "baud_rate", baud_rate_, 1);
  if (baud_status != CallbackReturn::SUCCESS) {
    return baud_status;
  }
  if (params.find("baud_rate") == params.end() && !use_dummy_) {
    RCLCPP_ERROR(logger(), "The 'baud_rate' hardware parameter is not set");
    return CallbackReturn::ERROR;
  }

  const auto read_tolerance_status =
    parse_int_param(params, "read_error_tolerance", read_error_tolerance_, 1);
  if (read_tolerance_status != CallbackReturn::SUCCESS) {
    return read_tolerance_status;
  }

  const auto write_tolerance_status =
    parse_int_param(params, "write_error_tolerance", write_error_tolerance_, 1);
  if (write_tolerance_status != CallbackReturn::SUCCESS) {
    return write_tolerance_status;
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

CallbackReturn DynamixelHardware::parse_int_param(
  const std::unordered_map<std::string, std::string> & params, const char * name, int & out,
  int min_value)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return CallbackReturn::SUCCESS;  // absent: caller decides whether that's required
  }
  try {
    out = std::stoi(it->second);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "Invalid '%s' hardware parameter '%s': %s", name, it->second.c_str(), e.what());
    return CallbackReturn::ERROR;
  }
  if (out < min_value) {
    RCLCPP_ERROR(logger(), "%s must be >= %d, got %d", name, min_value, out);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::parse_double_param(
  const std::unordered_map<std::string, std::string> & params, const char * name, double & out,
  bool allow_zero)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return CallbackReturn::SUCCESS;  // absent: caller decides whether that's required
  }
  try {
    out = std::stod(it->second);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "Invalid '%s' parameter '%s': %s", name, it->second.c_str(), e.what());
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(out)) {
    RCLCPP_ERROR(logger(), "%s must be finite, got '%s'", name, it->second.c_str());
    return CallbackReturn::ERROR;
  }
  if (!allow_zero && out == 0.0) {
    RCLCPP_ERROR(logger(), "%s must be non-zero", name);
    return CallbackReturn::ERROR;
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
    command_interfaces.emplace_back(
      std::make_shared<hardware_interface::CommandInterface>(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
    command_interfaces.emplace_back(
      std::make_shared<hardware_interface::CommandInterface>(
        info_.joints[i].name, kPwmInterfaceName, &joints_[i].command.pwm));
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
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, kPwmInterfaceName, &joints_[i].command.pwm));
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

  for (const auto & joint : joints_) {
    if (!driver_->ping(joint.id)) {
      RCLCPP_ERROR(logger(), "Failed to ping id %d: %s", joint.id, driver_->last_error().c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (!driver_->setup(all_ids())) {
    RCLCPP_ERROR(logger(), "Failed to set up handlers: %s", driver_->last_error().c_str());
    return CallbackReturn::ERROR;
  }

  // Every joint's configured mode is applied explicitly: Operating_Mode lives
  // in EEPROM, so a servo left in another mode by a previous run would stay
  // there. The driver's model-capability guard also rejects an impossible
  // configuration here, before any controller starts.
  for (auto & joint : joints_) {
    if (!driver_->set_control_mode(joint.id, joint.configured_mode)) {
      RCLCPP_ERROR(
        logger(), "Failed to set %s control for id %d: %s", mode_name(joint.configured_mode),
        joint.id, driver_->last_error().c_str());
      return CallbackReturn::ERROR;
    }
    joint.active_mode = joint.configured_mode;
  }
  legacy_mode_ = ControlMode::Position;

  std::vector<size_t> all_indices(joints_.size());
  std::iota(all_indices.begin(), all_indices.end(), 0);
  return write_extra_joint_params(all_indices);
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_activate");

  has_valid_state_ = false;
  consecutive_read_failures_ = 0;
  consecutive_write_failures_ = 0;

  // Best-effort initial read (#92): init_impl() seeds every joint state with
  // NaN, so a failure here must not let reset_command() copy that NaN into
  // the command and have write() sync-write it to servos that are about to
  // have torque enabled. Rather than failing activation outright,
  // read_joint_states() latches has_valid_state_ (and runs reset_command())
  // only on its first success, and the has_valid_state_ guard in write()
  // keeps the bus silent until that happens.
  if (!read_joint_states()) {
    RCLCPP_WARN(
      logger(), "Failed to read the initial joint states: %s", driver_->last_error().c_str());
  }

  if (set_torque_all(true) != return_type::OK) {
    return CallbackReturn::ERROR;
  }
  // Torque is on for every joint again, so a mode switch that failed before
  // must not keep write() in its error state.
  switch_failed_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(logger(), "on_deactivate");
  has_valid_state_ = false;
  if (set_torque_all(false) != return_type::OK) {
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
    // Every servo must still be tried when one of them fails, so this cannot
    // use set_torque_all(), which stops at the first failure.
    for (const auto & joint : joints_) {
      if (!driver_->set_torque(joint.id, false)) {
        RCLCPP_ERROR(
          logger(), "Failed to disable torque of id %d: %s", joint.id,
          driver_->last_error().c_str());
      }
    }
    torque_enabled_ = false;
    driver_->disconnect();
  }
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  pending_valid_ = false;

  // Prospective claim set per joint: the interfaces claimed right now, minus
  // the ones being stopped, plus the ones being started. On humble these
  // vectors carry the interfaces of ALL components, so entries that do not
  // name one of our joints are ignored (on jazzy+ the framework pre-filters
  // them, which makes this a no-op there).
  std::vector<std::set<std::string>> claims(joints_.size());
  for (size_t i = 0; i < joints_.size(); i++) {
    claims[i] = joints_[i].claimed_interfaces;
  }
  std::vector<bool> legacy(joints_.size(), false);

  const auto apply_keys =
    [this, &claims, &legacy](const std::vector<std::string> & keys, bool add) {
      for (const auto & key : keys) {
        const auto slash = key.rfind('/');
        if (slash == std::string::npos) {
          continue;
        }
        const int index = find_joint(key.substr(0, slash));
        if (index < 0) {
          continue;
        }
        const std::string interface_type = key.substr(slash + 1);
        if (interface_type != hardware_interface::HW_IF_POSITION &&
          interface_type != hardware_interface::HW_IF_VELOCITY &&
          interface_type != hardware_interface::HW_IF_EFFORT &&
          interface_type != kPwmInterfaceName)
        {
          if (!add) {
            // Releasing an interface this plugin does not know about changes
            // nothing; only a started one forces the legacy fallback.
            continue;
          }
          if (!unknown_interface_warned_) {
            RCLCPP_WARN(
              logger(), "Unknown command interface '%s'; keeping the legacy heuristic mode",
              key.c_str());
            unknown_interface_warned_ = true;
          }
          legacy[index] = true;
          continue;
        }
        if (add) {
          claims[index].insert(interface_type);
        } else {
          claims[index].erase(interface_type);
        }
      }
    };
  apply_keys(stop_interfaces, false);
  apply_keys(start_interfaces, true);

  pending_modes_.assign(joints_.size(), ControlMode::Position);
  pending_switch_.assign(joints_.size(), false);
  pending_legacy_.assign(joints_.size(), false);
  for (size_t i = 0; i < joints_.size(); i++) {
    if (legacy[i]) {
      pending_legacy_[i] = true;
      continue;
    }
    const auto & claimed = claims[i];
    if (claimed.empty()) {
      continue;  // nothing claimed: keep the current mode
    }
    const bool has_position = claimed.count(hardware_interface::HW_IF_POSITION) != 0;
    const bool has_velocity = claimed.count(hardware_interface::HW_IF_VELOCITY) != 0;
    const bool has_effort = claimed.count(hardware_interface::HW_IF_EFFORT) != 0;
    const bool has_pwm = claimed.count(kPwmInterfaceName) != 0;
    if (claimed.size() == 2 && has_position && has_velocity) {
      if (!legacy_warned_) {
        RCLCPP_WARN(
          logger(),
          "Joint '%s' claims both position and velocity; falling back to the legacy "
          "command-change heuristic (documented legacy behavior)",
          info_.joints[i].name.c_str());
        legacy_warned_ = true;
      }
      pending_legacy_[i] = true;
      continue;
    }
    ControlMode mode = ControlMode::Position;
    if (claimed.size() == 1 && has_position) {
      mode = is_position_family(joints_[i].configured_mode) ? joints_[i].configured_mode :
        ControlMode::Position;
    } else if (claimed.size() == 1 && has_velocity) {
      mode = ControlMode::Velocity;
    } else if (claimed.size() == 1 && has_effort) {
      mode = joints_[i].configured_mode == ControlMode::Torque ? ControlMode::Torque :
        ControlMode::Current;
    } else if (claimed.size() == 1 && has_pwm) {
      mode = ControlMode::PWM;
    } else if (claimed.size() == 2 && has_position && has_effort) {
      mode = ControlMode::CurrentBasedPosition;
    } else {
      std::string combination;
      for (const auto & interface_type : claimed) {
        combination += (combination.empty() ? "" : "+") + interface_type;
      }
      RCLCPP_ERROR(
        logger(), "Unsupported command interface combination '%s' for joint '%s'",
        combination.c_str(), info_.joints[i].name.c_str());
      return return_type::ERROR;
    }
    pending_modes_[i] = mode;
    pending_switch_[i] = true;
  }
  pending_claims_ = std::move(claims);
  pending_valid_ = true;
  return return_type::OK;
}

return_type DynamixelHardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */)
{
  if (!pending_valid_) {
    return return_type::OK;
  }
  pending_valid_ = false;

  std::vector<size_t> indices;
  std::vector<ControlMode> modes;
  for (size_t i = 0; i < joints_.size(); i++) {
    if (pending_legacy_[i]) {
      if (joints_[i].active_mode != legacy_mode_) {
        indices.push_back(i);
        modes.push_back(legacy_mode_);
      }
    } else if (pending_switch_[i] && pending_modes_[i] != joints_[i].active_mode) {
      indices.push_back(i);
      modes.push_back(pending_modes_[i]);
    }
  }
  if (apply_mode_switch(indices, modes) != return_type::OK) {
    // ControllerManager only logs a non-OK return here and starts the
    // controller anyway, so the failure has to keep being reported: the
    // affected joints were de-energized by the torque-off leg and nothing
    // else would ever surface the fault. write() escalates while the latch is
    // set, which routes the component into on_error() -- the existing path
    // that disables torque everywhere and disconnects.
    switch_failed_ = true;
    RCLCPP_ERROR(
      logger(),
      "Command mode switch failed; the affected joints are de-energized. write() will report an "
      "error until torque is restored by a successful switch or by re-activating the component");
    return return_type::ERROR;
  }
  if (torque_enabled_ || !torque_enable_param_) {
    // Only an energized outcome clears the fault. apply_mode_switch() also
    // returns OK when it touched no torque at all -- an empty switch, or one
    // performed while the servos are already de-energized by a previous
    // failure -- and clearing on those would report a limp joint as healthy
    // again, which is exactly what the latch exists to prevent. The
    // exception is torque_enable_param_ == false: there, staying de-energized
    // IS the intended healthy outcome (that is the whole point of the
    // parameter), so a successful switch is the best result available and
    // must clear the latch -- the plain torque_enabled_ check still governs
    // the normal (torque_enable_param_ true) configuration, where clearing on
    // a de-energized outcome would misreport a limp joint as healthy.
    switch_failed_ = false;
  }
  // The claim bookkeeping is only committed once the servos accepted the
  // switch, so a rejected switch does not leave the plugin acting on claims
  // it never applied.
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].claimed_interfaces = pending_claims_[i];
    joints_[i].legacy = pending_legacy_[i];
  }
  return return_type::OK;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // Transient sync-read failures (noisy bus, momentary dropout) hold the
  // last-known state and report OK; only read_error_tolerance_ consecutive
  // failures escalate to ERROR so the controller manager can react (#88).
  // Any success resets the counter.
  if (!read_joint_states()) {
    ++consecutive_read_failures_;
    RCLCPP_WARN(
      logger(), "read_states failed (%d/%d): %s", consecutive_read_failures_,
      read_error_tolerance_, driver_->last_error().c_str());
    if (consecutive_read_failures_ >= read_error_tolerance_) {
      RCLCPP_ERROR(logger(), "read_states failure tolerance exceeded, reporting ERROR");
      return return_type::ERROR;
    }
    return return_type::OK;  // hold last-known state
  }
  consecutive_read_failures_ = 0;
  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  if (switch_failed_) {
    // Already logged once by perform_command_mode_switch(); commanding
    // de-energized servos would only pretend the cycle was healthy.
    return return_type::ERROR;
  }

  if (!has_valid_state_) {
    // #92: never send commands derived from the NaN/zero state init_impl()
    // seeds every joint with. write() stays silent until the first
    // successful read after activation latches has_valid_state_ (see
    // read_joint_states()).
    return return_type::OK;
  }

  driver_->tick(period.seconds());

  // A failed mode switch leaves the servos with torque off, so it must not be
  // reported to the controller manager as a successful cycle.
  if (update_legacy_heuristic() != return_type::OK) {
    return return_type::ERROR;
  }

  // Joints are batched per active mode so that servos in different modes can
  // be driven in the same cycle (#69).
  std::vector<uint8_t> position_ids;
  std::vector<double> position_commands;
  std::vector<uint8_t> velocity_ids;
  std::vector<double> velocity_commands;
  std::vector<uint8_t> effort_ids;
  std::vector<double> effort_commands;
  std::vector<uint8_t> pwm_ids;
  std::vector<double> pwm_commands;

  for (size_t i = 0; i < joints_.size(); i++) {
    auto & joint = joints_[i];
    switch (joint.active_mode) {
      case ControlMode::Position:
      case ControlMode::ExtendedPosition:
      case ControlMode::MultiTurn:
        position_ids.push_back(joint.id);
        position_commands.push_back(to_motor_position(i, joint.command.position));
        joint.prev_command.position = joint.command.position;
        break;
      case ControlMode::CurrentBasedPosition:
        position_ids.push_back(joint.id);
        position_commands.push_back(to_motor_position(i, joint.command.position));
        joint.prev_command.position = joint.command.position;
        // The current cap is only commanded when the controller claimed the
        // effort interface; otherwise the servo keeps its Goal_Current
        // register (which defaults to Current_Limit).
        if (joint.claimed_interfaces.count(hardware_interface::HW_IF_EFFORT) != 0) {
          effort_ids.push_back(joint.id);
          effort_commands.push_back(to_motor_effort(i, effort_command_to_motor(i)));
          joint.prev_command.effort = joint.command.effort;
        }
        break;
      case ControlMode::Velocity:
        velocity_ids.push_back(joint.id);
        velocity_commands.push_back(to_motor_velocity(i, joint.command.velocity));
        joint.prev_command.velocity = joint.command.velocity;
        break;
      case ControlMode::Current:
      case ControlMode::Torque:
        effort_ids.push_back(joint.id);
        effort_commands.push_back(to_motor_effort(i, effort_command_to_motor(i)));
        joint.prev_command.effort = joint.command.effort;
        break;
      case ControlMode::PWM:
        // PWM duty ratios are never gear-converted (#95/#94): they are not a
        // physical position/velocity/effort quantity.
        pwm_ids.push_back(joint.id);
        pwm_commands.push_back(joint.command.pwm);
        joint.prev_command.pwm = joint.command.pwm;
        break;
    }
  }

  // `driver_->write_x(...) && ok` (not `ok && ...`): every batch is attempted
  // even when an earlier one fails, so one bad group cannot stall the others.
  bool ok = true;
  if (!position_ids.empty()) {
    ok = driver_->write_positions(position_ids, position_commands) && ok;
  }
  if (!velocity_ids.empty()) {
    ok = driver_->write_velocities(velocity_ids, velocity_commands) && ok;
  }
  if (!effort_ids.empty()) {
    ok = driver_->write_efforts(effort_ids, effort_commands) && ok;
  }
  if (!pwm_ids.empty()) {
    ok = driver_->write_pwms(pwm_ids, pwm_commands) && ok;
  }
  return handle_write_result(ok);
}

return_type DynamixelHardware::handle_write_result(const bool ok)
{
  // Mirrors read()'s tolerance handling (#88): a transient driver write_*()
  // failure holds at OK, and only write_error_tolerance_ consecutive
  // failures escalate to ERROR. Any success resets the counter.
  if (ok) {
    consecutive_write_failures_ = 0;
    return return_type::OK;
  }
  ++consecutive_write_failures_;
  RCLCPP_WARN(
    logger(), "driver write failed (%d/%d): %s", consecutive_write_failures_,
    write_error_tolerance_, driver_->last_error().c_str());
  if (consecutive_write_failures_ >= write_error_tolerance_) {
    RCLCPP_ERROR(logger(), "write failure tolerance exceeded, reporting ERROR");
    return return_type::ERROR;
  }
  return return_type::OK;
}

std::vector<uint8_t> DynamixelHardware::all_ids() const
{
  std::vector<uint8_t> ids;
  ids.reserve(joints_.size());
  for (const auto & joint : joints_) {
    ids.push_back(joint.id);
  }
  return ids;
}

int DynamixelHardware::find_joint(const std::string & joint_name) const
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].name == joint_name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool DynamixelHardware::read_joint_states()
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  if (!driver_->read_states(all_ids(), positions, velocities, efforts)) {
    return false;
  }
  for (size_t i = 0; i < joints_.size(); i++) {
    joints_[i].state.position = to_joint_position(i, positions[i]);
    joints_[i].state.velocity = to_joint_velocity(i, velocities[i]);
    // torque_constant (mA -> Nm) and gear_ratio commute, so their order here
    // is arbitrary; effort_state_from_motor stays the driver-boundary
    // conversion, gear composes around it.
    joints_[i].state.effort = to_joint_effort(i, effort_state_from_motor(i, efforts[i]));
  }
  if (!has_valid_state_) {
    // First successful state read since activation (#92): latch so write()
    // starts sending commands, and re-sync commands from the now-known state
    // so they never carry the NaN/zero placeholder init_impl() seeds every
    // joint with. Both read() and on_activate() call this helper, so the
    // latch fires from either call site without duplicating this logic.
    has_valid_state_ = true;
    reset_command();
  }
  return true;
}

return_type DynamixelHardware::set_torque_all(const bool enabled)
{
  if (enabled && !torque_enable_param_) {
    RCLCPP_DEBUG(logger(), "torque_enable is false: skipping torque on");
    return return_type::OK;
  }
  for (const auto & joint : joints_) {
    if (!driver_->set_torque(joint.id, enabled)) {
      RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
      return return_type::ERROR;
    }
  }
  if (enabled && !torque_enabled_) {
    reset_command();
    RCLCPP_INFO(logger(), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    RCLCPP_INFO(logger(), "Torque disabled");
  }
  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::apply_mode_switch(
  const std::vector<size_t> & indices, const std::vector<ControlMode> & modes)
{
  if (indices.empty()) {
    return return_type::OK;
  }
  // Dynamixel requirement: the operating mode can only change with torque off.
  const bool was_torque_enabled = torque_enabled_;
  if (was_torque_enabled) {
    // torque_enabled_ is a single hardware-wide approximation (per-joint
    // torque state is a separate milestone), so it is maintained to follow the
    // leg in flight. While the servos being switched are de-energized it must
    // read false: a stale `true` would make write() sync-write goals to limp
    // servos and would make the next switch believe it still has to cycle
    // torque. Joints outside `indices` may still be energized -- that is
    // exactly why the re-enable leg below sets the flag before its loop.
    torque_enabled_ = false;
    for (const auto index : indices) {
      if (!driver_->set_torque(joints_[index].id, false)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
    }
  }
  for (size_t k = 0; k < indices.size(); k++) {
    auto & joint = joints_[indices[k]];
    if (!driver_->set_control_mode(joint.id, modes[k])) {
      RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
      return return_type::ERROR;
    }
    joint.active_mode = modes[k];
    RCLCPP_INFO(
      logger(), "Joint '%s' switched to %s control", info_.joints[indices[k]].name.c_str(),
      mode_name(modes[k]));
  }
  // Extra control-table parameters live in RAM and are reset by a mode change.
  if (write_extra_joint_params(indices) != CallbackReturn::SUCCESS) {
    return return_type::ERROR;
  }
  if (was_torque_enabled && torque_enable_param_) {
    // Set before the loop, not after it: a partial failure still leaves the
    // servos the loop already reached energized, and under-reporting that
    // would make the next switch skip the mandatory torque-off leg and try to
    // rewrite Operating_Mode on a torqued servo, which the firmware refuses.
    torque_enabled_ = true;
    for (const auto index : indices) {
      if (!driver_->set_torque(joints_[index].id, true)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
    }
  }
  for (const auto index : indices) {
    reset_joint_command(index);
  }
  return return_type::OK;
}

return_type DynamixelHardware::update_legacy_heuristic()
{
  // Kept from the pre-M3 implementation for joints whose controller claims
  // position and velocity together: a changed velocity command switches them
  // to velocity control, else a changed position command switches them to
  // position control, else they stay in the current mode.
  std::vector<size_t> legacy_indices;
  for (size_t i = 0; i < joints_.size(); i++) {
    if (joints_[i].legacy) {
      legacy_indices.push_back(i);
    }
  }
  if (legacy_indices.empty()) {
    return return_type::OK;
  }
  bool velocity_changed = false;
  bool position_changed = false;
  for (const auto i : legacy_indices) {
    velocity_changed =
      velocity_changed || joints_[i].command.velocity != joints_[i].prev_command.velocity;
    position_changed =
      position_changed || joints_[i].command.position != joints_[i].prev_command.position;
  }
  ControlMode target = legacy_mode_;
  if (velocity_changed) {
    target = ControlMode::Velocity;
  } else if (position_changed) {
    target = ControlMode::Position;
  }
  // Every legacy joint must be checked, not just the first one: a partially
  // failed switch can leave the group's active modes diverged, and the
  // trailing joints still need to be re-synced.
  const bool all_in_target = std::all_of(
    legacy_indices.cbegin(), legacy_indices.cend(),
    [this, target](size_t i) {return joints_[i].active_mode == target;});
  if (target == legacy_mode_ && all_in_target) {
    return return_type::OK;
  }
  const std::vector<ControlMode> modes(legacy_indices.size(), target);
  const auto result = apply_mode_switch(legacy_indices, modes);
  if (result == return_type::OK) {
    legacy_mode_ = target;
  }
  return result;
}

CallbackReturn DynamixelHardware::write_extra_joint_params(const std::vector<size_t> & indices)
{
  for (const auto index : indices) {
    const auto & joint_params = info_.joints[index].parameters;
    for (const auto * param_name : kExtraJointParameters) {
      const auto it = joint_params.find(param_name);
      if (it == joint_params.end()) {
        continue;
      }
      int value = 0;
      try {
        value = std::stoi(it->second);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          logger(), "Joint '%s' has an invalid '%s' parameter '%s': %s",
          info_.joints[index].name.c_str(), param_name, it->second.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
      if (!driver_->write_item(joints_[index].id, param_name, value)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(
        logger(), "%s set to %d for joint '%s'", param_name, value,
        info_.joints[index].name.c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

void DynamixelHardware::reset_command()
{
  for (size_t i = 0; i < joints_.size(); i++) {
    reset_joint_command(i);
  }
}

void DynamixelHardware::reset_joint_command(size_t index)
{
  auto & joint = joints_[index];
  joint.command.position = joint.state.position;
  joint.command.velocity = 0.0;
  joint.command.effort = 0.0;
  joint.command.pwm = 0.0;
  joint.prev_command = joint.command;
}

double DynamixelHardware::effort_command_to_motor(size_t index) const
{
  const auto & joint = joints_[index];
  if (joint.torque_constant > 0.0) {
    return joint.command.effort * 1000.0 / joint.torque_constant;  // Nm -> mA
  }
  return joint.command.effort;  // already mA
}

double DynamixelHardware::effort_state_from_motor(size_t index, double motor_effort) const
{
  const auto & joint = joints_[index];
  if (joint.torque_constant > 0.0) {
    return motor_effort / 1000.0 * joint.torque_constant;  // mA -> Nm
  }
  return motor_effort;  // stays mA
}

double DynamixelHardware::to_joint_position(size_t index, double motor_position) const
{
  const auto & joint = joints_[index];
  return motor_position / joint.gear_ratio - joint.offset;
}

double DynamixelHardware::to_motor_position(size_t index, double joint_position) const
{
  const auto & joint = joints_[index];
  return (joint_position + joint.offset) * joint.gear_ratio;
}

double DynamixelHardware::to_joint_velocity(size_t index, double motor_velocity) const
{
  return motor_velocity / joints_[index].gear_ratio;
}

double DynamixelHardware::to_motor_velocity(size_t index, double joint_velocity) const
{
  return joint_velocity * joints_[index].gear_ratio;
}

double DynamixelHardware::to_joint_effort(size_t index, double motor_effort) const
{
  return motor_effort * joints_[index].gear_ratio;
}

double DynamixelHardware::to_motor_effort(size_t index, double joint_effort) const
{
  return joint_effort / joints_[index].gear_ratio;
}

bool DynamixelHardware::is_position_family(ControlMode mode)
{
  return mode == ControlMode::Position || mode == ControlMode::ExtendedPosition ||
         mode == ControlMode::MultiTurn || mode == ControlMode::CurrentBasedPosition;
}

bool DynamixelHardware::parse_control_mode(const std::string & value, ControlMode & mode)
{
  if (value == "position") {
    mode = ControlMode::Position;
  } else if (value == "extended_position") {
    mode = ControlMode::ExtendedPosition;
  } else if (value == "multi_turn") {
    mode = ControlMode::MultiTurn;
  } else if (value == "current_based_position") {
    mode = ControlMode::CurrentBasedPosition;
  } else if (value == "velocity") {
    mode = ControlMode::Velocity;
  } else if (value == "current") {
    mode = ControlMode::Current;
  } else if (value == "torque") {
    mode = ControlMode::Torque;
  } else if (value == "pwm") {
    mode = ControlMode::PWM;
  } else {
    return false;
  }
  return true;
}

const char * DynamixelHardware::mode_name(ControlMode mode)
{
  switch (mode) {
    case ControlMode::Position:
      return "position";
    case ControlMode::Velocity:
      return "velocity";
    case ControlMode::Current:
      return "current";
    case ControlMode::Torque:
      return "torque";
    case ControlMode::ExtendedPosition:
      return "extended_position";
    case ControlMode::MultiTurn:
      return "multi_turn";
    case ControlMode::CurrentBasedPosition:
      return "current_based_position";
    case ControlMode::PWM:
      return "pwm";
  }
  return "unknown";
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
