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
/// A control-table item a <joint> may set by name, and where it lives on the
/// servo. RAM items are reset to their defaults by an operating-mode change,
/// so they have to be rewritten after every switch; EEPROM items survive one
/// and are written once, at configuration time.
struct ExtraJointParameter
{
  const char * name;
  bool ram;
};

constexpr ExtraJointParameter kExtraJointParameters[] = {
  {"Profile_Velocity", true},
  {"Profile_Acceleration", true},
  {"Position_P_Gain", true},
  {"Position_I_Gain", true},
  {"Position_D_Gain", true},
  {"Velocity_P_Gain", true},
  {"Velocity_I_Gain", true},
  // X-series control-table address 9, in the EEPROM area -- as is
  // Operating_Mode at 11, which is why on_configure() has to set the mode
  // explicitly (see there).
  {"Return_Delay_Time", false},
};

namespace
{
/// Denominator of the failure-tolerance warnings: the configured limit, or
/// "disabled" when it is 0 (failures are still counted and warned about, they
/// just never escalate), which reads better than a bare "/0".
std::string tolerance_text(int tolerance)
{
  return tolerance > 0 ? std::to_string(tolerance) : std::string("disabled");
}
}  // namespace

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
    // id is required, unlike the optional parameters parse_int_param() is
    // normally used for -- so presence is checked above by hand, and once
    // that passes, parse_int_param() (min_value 0) is reused for the
    // non-numeric and negative cases so their error wording matches every
    // other hardware parameter. The upper bound is intentionally a separate
    // check: parse_int_param() only expresses a floor, and folding a ceiling
    // into it for this one caller would complicate a helper every other
    // parameter uses for a rule id alone needs. static_cast<uint8_t> is only
    // safe to apply once parsed_id is confirmed inside [0, kMaxDynamixelId].
    int parsed_id = 0;
    const auto id_status =
      parse_int_param(joint_params, "id", parsed_id, 0, joint_info.name.c_str());
    if (id_status != CallbackReturn::SUCCESS) {
      return id_status;
    }
    if (parsed_id > kMaxDynamixelId) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid 'id' parameter: must be <= %d, got %d",
        joint_info.name.c_str(), kMaxDynamixelId, parsed_id);
      return CallbackReturn::ERROR;
    }
    joint.id = static_cast<uint8_t>(parsed_id);

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

    // Strictly positive and finite: std::stod (via parse_double_param) parses
    // "nan"/"inf"/"-inf" per strtod, and both silently corrupt the effort
    // conversion downstream (effort_command_to_motor / effort_state_from_motor)
    // if let through -- NaN degrades the joint to raw-mA effort with no
    // warning, and inf zeroes every effort command and publishes +-inf/NaN on
    // the effort state.
    const auto torque_constant_status = parse_double_param(
      joint_params, "torque_constant", joint.torque_constant, DoubleParamRule::kFinitePositive,
      joint_info.name.c_str());
    if (torque_constant_status != CallbackReturn::SUCCESS) {
      return torque_constant_status;
    }

    // Zero would make the gear conversion divide by zero; negative ratios are
    // deliberately allowed -- they invert the rotation direction (#95/#94).
    const auto gear_ratio_status = parse_double_param(
      joint_params, "gear_ratio", joint.gear_ratio, DoubleParamRule::kFiniteNonZero,
      joint_info.name.c_str());
    if (gear_ratio_status != CallbackReturn::SUCCESS) {
      return gear_ratio_status;
    }

    // Joint-side position offset (#96/#93); any finite value, including
    // zero (the default, a no-op) or negative, is valid.
    const auto offset_status = parse_double_param(
      joint_params, "offset", joint.offset, DoubleParamRule::kFinite, joint_info.name.c_str());
    if (offset_status != CallbackReturn::SUCCESS) {
      return offset_status;
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

  // Two joints sharing one Dynamixel id would each get an independent
  // active_mode/command/state record for the same physical servo, fighting
  // each other on every write cycle -- reject before any I/O is attempted.
  for (size_t i = 0; i < joints_.size(); i++) {
    for (size_t j = i + 1; j < joints_.size(); j++) {
      if (joints_[i].id == joints_[j].id) {
        RCLCPP_ERROR(
          logger(), "Joints '%s' and '%s' both use Dynamixel id %d; each joint must address a "
          "distinct servo",
          info.joints[i].name.c_str(), info.joints[j].name.c_str(), joints_[i].id);
        return CallbackReturn::ERROR;
      }
    }
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

  // Both tolerances accept 0, which disables escalation for that direction.
  // Escalating to return_type::ERROR is new behavior (read() used to log and
  // return OK forever, write() returned OK regardless of the driver's result),
  // and it lands on every existing user without any URDF change -- so the
  // operator of a marginal USB adapter or an electrically noisy bus needs a
  // configuration that restores the old ride-through behavior. Negative values
  // stay rejected: they express nothing 0 does not.
  const auto read_tolerance_status =
    parse_int_param(params, "read_error_tolerance", read_error_tolerance_, 0);
  if (read_tolerance_status != CallbackReturn::SUCCESS) {
    return read_tolerance_status;
  }

  const auto write_tolerance_status =
    parse_int_param(params, "write_error_tolerance", write_error_tolerance_, 0);
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
  int min_value, const char * joint_name)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return CallbackReturn::SUCCESS;  // absent: caller decides whether that's required
  }
  try {
    out = std::stoi(it->second);
  } catch (const std::exception & e) {
    if (joint_name) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid '%s' parameter '%s': %s", joint_name, name,
        it->second.c_str(), e.what());
    } else {
      RCLCPP_ERROR(
        logger(), "Invalid '%s' hardware parameter '%s': %s", name, it->second.c_str(), e.what());
    }
    return CallbackReturn::ERROR;
  }
  if (out < min_value) {
    if (joint_name) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid '%s' parameter: must be >= %d, got %d", joint_name,
        name, min_value, out);
    } else {
      RCLCPP_ERROR(logger(), "%s must be >= %d, got %d", name, min_value, out);
    }
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::parse_double_param(
  const std::unordered_map<std::string, std::string> & params, const char * name, double & out,
  DoubleParamRule rule, const char * joint_name)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return CallbackReturn::SUCCESS;  // absent: caller decides whether that's required
  }
  try {
    out = std::stod(it->second);
  } catch (const std::exception & e) {
    if (joint_name) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid '%s' parameter '%s': %s", joint_name, name,
        it->second.c_str(), e.what());
    } else {
      RCLCPP_ERROR(
        logger(), "Invalid '%s' parameter '%s': %s", name, it->second.c_str(), e.what());
    }
    return CallbackReturn::ERROR;
  }
  // std::stod parses "nan"/"inf"/"-inf" per strtod, so finiteness is not
  // implied by a successful parse and has to be checked on its own,
  // regardless of which rule below further restricts the value.
  if (!std::isfinite(out)) {
    if (joint_name) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid '%s' parameter: must be finite, got '%s'",
        joint_name, name, it->second.c_str());
    } else {
      RCLCPP_ERROR(logger(), "%s must be finite, got '%s'", name, it->second.c_str());
    }
    return CallbackReturn::ERROR;
  }
  if (rule == DoubleParamRule::kFiniteNonZero && out == 0.0) {
    if (joint_name) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has an invalid '%s' parameter: must be non-zero", joint_name, name);
    } else {
      RCLCPP_ERROR(logger(), "%s must be non-zero", name);
    }
    return CallbackReturn::ERROR;
  }
  if (rule == DoubleParamRule::kFinitePositive && out <= 0.0) {
    // Wording kept close to this check's original single-caller form
    // (torque_constant, Nm/A): it already names the joint and the unit.
    if (joint_name) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has a non-positive '%s' parameter '%s' (Nm/A > 0)", joint_name,
        name, it->second.c_str());
    } else {
      RCLCPP_ERROR(logger(), "%s must be positive (Nm/A > 0), got '%s'", name, it->second.c_str());
    }
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
  // Configuration is the one point where the servo's EEPROM contents are not
  // known to match the URDF, so everything is written here, RAM and EEPROM
  // alike.
  return write_extra_joint_params(all_indices, ExtraParamScope::kAll);
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
    for (auto & joint : joints_) {
      // Cleared before the call, as in apply_mode_switch(): the component is
      // on its way down and a stale `true` would report a servo this path
      // just cut power to as still energized.
      joint.torque_enabled = false;
      if (!driver_->set_torque(joint.id, false)) {
        RCLCPP_ERROR(
          logger(), "Failed to disable torque of id %d: %s", joint.id,
          driver_->last_error().c_str());
      }
    }
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
  if (apply_mode_switch_or_latch(indices, modes) != return_type::OK) {
    return return_type::ERROR;
  }
  if (all_torque_enabled() || !torque_enable_param_) {
    // Only a fully energized outcome clears the fault. apply_mode_switch()
    // also returns OK when it touched no torque at all -- an empty switch, or
    // one performed while the servos are already de-energized by a previous
    // failure -- and clearing on those would report a limp joint as healthy
    // again, which is exactly what the latch exists to prevent. Every joint
    // has to be energized, not just the ones this switch touched: a joint
    // left limp by an earlier failure is still limp. The exception is
    // torque_enable_param_ == false: there, staying de-energized IS the
    // intended healthy outcome (that is the whole point of the parameter), so
    // a successful switch is the best result available and must clear the
    // latch -- the all_torque_enabled() check still governs the normal
    // (torque_enable_param_ true) configuration, where clearing on a
    // de-energized outcome would misreport a limp joint as healthy.
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
  // Any success resets the counter. A tolerance of 0 disables the escalation
  // (see init_impl()); failures are still counted and warned about, because
  // going silent about a dying bus would be worse than the old behavior it
  // restores.
  if (!read_joint_states()) {
    // Saturating: with escalation disabled nothing but a successful read ever
    // resets this counter, so a permanently dead bus would otherwise overflow
    // it -- signed overflow is undefined behavior -- after a few months of
    // 100 Hz cycles.
    if (consecutive_read_failures_ < std::numeric_limits<int>::max()) {
      ++consecutive_read_failures_;
    }
    RCLCPP_WARN(
      logger(), "read_states failed (%d/%s): %s", consecutive_read_failures_,
      tolerance_text(read_error_tolerance_).c_str(), driver_->last_error().c_str());
    if (read_error_tolerance_ > 0 && consecutive_read_failures_ >= read_error_tolerance_) {
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

  // Everything above this point still runs with torque_enable false: tick()
  // drives the dummy driver's integration, the legacy heuristic keeps the
  // servos in the right operating mode, and the batching loop above records
  // this cycle's commands as prev_command -- which is what the heuristic
  // compares against next cycle. Only the goal sync-writes are skipped,
  // because no joint can ever be energized in this configuration
  // (set_torque_all() returns early and apply_mode_switch()'s re-enable leg is
  // gated), so every one of them is a bus round-trip the servo cannot act on
  // and whose stored goal reset_joint_command() would overwrite anyway. The
  // configuration this parameter exists for -- a back-driven leader arm --
  // wants that bus budget spent on reads (#90).
  //
  // The write-failure counter is deliberately left untouched rather than
  // reset through handle_write_result(true): a cycle that issued no driver
  // call is no evidence the bus is healthy, just as it is no failure. This
  // matches the has_valid_state_ guard above, which returns OK the same way.
  if (!torque_enable_param_) {
    return return_type::OK;
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
  // failures escalate to ERROR. Any success resets the counter, and a
  // tolerance of 0 disables the escalation while still counting and warning.
  if (ok) {
    consecutive_write_failures_ = 0;
    return return_type::OK;
  }
  if (consecutive_write_failures_ < std::numeric_limits<int>::max()) {
    ++consecutive_write_failures_;  // saturating, as in read()
  }
  RCLCPP_WARN(
    logger(), "driver write failed (%d/%s): %s", consecutive_write_failures_,
    tolerance_text(write_error_tolerance_).c_str(), driver_->last_error().c_str());
  if (write_error_tolerance_ > 0 && consecutive_write_failures_ >= write_error_tolerance_) {
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
  bool any_transitioned = false;
  for (size_t i = 0; i < joints_.size(); i++) {
    auto & joint = joints_[i];
    if (!driver_->set_torque(joint.id, enabled)) {
      RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
      return return_type::ERROR;
    }
    // Recorded as each call returns, so the early return above leaves the
    // joints this sweep already reached reporting what actually happened to
    // them rather than the state the sweep was aiming for.
    if (joint.torque_enabled == enabled) {
      continue;
    }
    joint.torque_enabled = enabled;
    any_transitioned = true;
    if (enabled) {
      // Commands are re-synced from the current state before the servo can
      // act on them, so torque coming on never replays a stale goal.
      reset_joint_command(i);
    }
  }
  // One line for the whole sweep, and only when something actually changed:
  // per joint this would spam a 5-servo arm's log on every activation.
  if (any_transitioned) {
    RCLCPP_INFO(logger(), "%s", enabled ? "Torque enabled" : "Torque disabled");
  }
  return return_type::OK;
}

bool DynamixelHardware::all_torque_enabled() const
{
  return std::all_of(
    joints_.cbegin(), joints_.cend(), [](const Joint & joint) {return joint.torque_enabled;});
}

return_type DynamixelHardware::apply_mode_switch(
  const std::vector<size_t> & indices, const std::vector<ControlMode> & modes)
{
  if (indices.empty()) {
    return return_type::OK;
  }
  // Dynamixel requirement: the operating mode can only change with torque off.
  // Every joint being switched is de-energized unconditionally, including one
  // this plugin already believes is off: `torque_enabled` false only means
  // torque was never confirmed on, and a servo whose torque-on was rejected
  // may well be energized anyway. Sending the redundant torque-off is
  // idempotent, whereas skipping a needed one has the firmware refuse the
  // Operating_Mode write.
  //
  // The joints that were confirmed energized on entry are remembered, because
  // only those may be restored afterwards -- re-energizing the rest would
  // power up servos that a deactivation, a torque_enable=false configuration
  // or an earlier failure deliberately left limp. Joints outside `indices` are
  // never touched at all.
  std::vector<size_t> to_restore;
  to_restore.reserve(indices.size());
  for (const auto index : indices) {
    auto & joint = joints_[index];
    if (joint.torque_enabled) {
      to_restore.push_back(index);
    }
    // Cleared before the call, not after: from here on the servo is being
    // de-energized, and a stale `true` would make write() sync-write goals to
    // a limp servo and would let all_torque_enabled() report it as healthy.
    joint.torque_enabled = false;
    if (!driver_->set_torque(joint.id, false)) {
      RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
      return return_type::ERROR;
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
  // A mode change resets the RAM control-table parameters to their defaults,
  // so those are rewritten here. The EEPROM ones (Return_Delay_Time) survive
  // it untouched: rewriting them would be one more blocking per-joint
  // round-trip, on a path write() can reach, to restore a value that was
  // never lost.
  if (write_extra_joint_params(indices, ExtraParamScope::kRamOnly) != CallbackReturn::SUCCESS) {
    return return_type::ERROR;
  }
  if (torque_enable_param_) {
    for (const auto index : to_restore) {
      auto & joint = joints_[index];
      if (!driver_->set_torque(joint.id, true)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return return_type::ERROR;
      }
      // Recorded only once the driver confirms it, exactly like
      // set_torque_all(): a rejected torque-on is no evidence the servo came
      // back up, and claiming otherwise would let all_torque_enabled() clear
      // the mode-switch fault latch while that joint is still limp. The
      // torque-off leg above no longer skips anything, so under-reporting
      // here cannot cost a joint its mandatory torque-off.
      joint.torque_enabled = true;
    }
  }
  for (const auto index : indices) {
    reset_joint_command(index);
  }
  return return_type::OK;
}

return_type DynamixelHardware::apply_mode_switch_or_latch(
  const std::vector<size_t> & indices, const std::vector<ControlMode> & modes)
{
  if (apply_mode_switch(indices, modes) != return_type::OK) {
    // ControllerManager only logs a non-OK return here and starts the
    // controller anyway, so the failure has to keep being reported: the
    // affected joints were de-energized by the torque-off leg and nothing
    // else would ever surface the fault. write() escalates while the latch is
    // set, which routes the component into on_error() -- the existing path
    // that disables torque everywhere and disconnects. Both
    // perform_command_mode_switch() and update_legacy_heuristic() route
    // through here, so a failure on either path latches identically -- the
    // legacy path used to report return_type::ERROR for a single cycle
    // without latching, so write() silently re-drove the already-de-energized
    // servo through the driver on every later cycle (#112 follow-up).
    switch_failed_ = true;
    RCLCPP_ERROR(
      logger(),
      "Command mode switch failed; the affected joints are de-energized. Re-activate the "
      "component to energize them again -- a later mode switch will not, because restoring "
      "torque is limited to joints that were energized when the switch began. write() reports "
      "an error until then. (With torque_enable=false, de-energized is the intended state and "
      "a successful switch clears the error.)");
    return return_type::ERROR;
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
  const auto result = apply_mode_switch_or_latch(legacy_indices, modes);
  if (result == return_type::OK) {
    legacy_mode_ = target;
  }
  return result;
}

CallbackReturn DynamixelHardware::write_extra_joint_params(
  const std::vector<size_t> & indices, ExtraParamScope scope)
{
  for (const auto index : indices) {
    const auto & joint_params = info_.joints[index].parameters;
    for (const auto & extra_param : kExtraJointParameters) {
      if (scope == ExtraParamScope::kRamOnly && !extra_param.ram) {
        continue;
      }
      const auto it = joint_params.find(extra_param.name);
      if (it == joint_params.end()) {
        continue;
      }
      int value = 0;
      try {
        value = std::stoi(it->second);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          logger(), "Joint '%s' has an invalid '%s' parameter '%s': %s",
          info_.joints[index].name.c_str(), extra_param.name, it->second.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
      if (!driver_->write_item(joints_[index].id, extra_param.name, value)) {
        RCLCPP_FATAL(logger(), "%s", driver_->last_error().c_str());
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(
        logger(), "%s set to %d for joint '%s'", extra_param.name, value,
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
