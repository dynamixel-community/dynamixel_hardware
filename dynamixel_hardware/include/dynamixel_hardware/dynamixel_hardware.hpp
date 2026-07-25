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

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dynamixel_hardware/compat.hpp"
#include "dynamixel_hardware/dynamixel_driver.hpp"
#include "dynamixel_hardware/visibility_control.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"

#if DXL_HAS_PARAMS_ON_INIT
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#endif

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace dynamixel_hardware
{
/// Name of the custom PWM command interface (URDF <command_interface name="pwm"/>).
constexpr char kPwmInterfaceName[] = "pwm";

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double pwm{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
  uint8_t id{0};
  /// Mode from the URDF 'control_mode' parameter, applied in on_configure().
  ControlMode configured_mode{ControlMode::Position};
  /// Mode the servo is believed to be in right now.
  ControlMode active_mode{ControlMode::Position};
  /// Claimed position and velocity together -> legacy write() heuristic.
  bool legacy{false};
  /// Nm/A; 0.0 means unset and the effort interfaces carry milliamps.
  double torque_constant{0.0};
  std::set<std::string> claimed_interfaces{};
};

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

#if DXL_HAS_PARAMS_ON_INIT
  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
#endif

#if DXL_HAS_ON_EXPORT
  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;
#else
  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#endif

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// For tests and derived drivers: replace the driver. An injected driver
  /// survives on_init (init_impl only creates a driver when none is set).
  DYNAMIXEL_HARDWARE_PUBLIC
  void set_driver_for_testing(std::unique_ptr<DynamixelDriver> driver);

private:
  CallbackReturn init_impl(const hardware_interface::HardwareInfo & info);

  rclcpp::Logger logger() const;

  std::vector<uint8_t> all_ids() const;
  int find_joint(const std::string & joint_name) const;

  bool read_joint_states();
  return_type handle_write_result(const bool ok);
  return_type set_torque_all(const bool enabled);
  /// Torque off -> set_control_mode -> extra-parameter rewrite -> torque on.
  return_type apply_mode_switch(
    const std::vector<size_t> & indices, const std::vector<ControlMode> & modes);
  return_type update_legacy_heuristic();
  CallbackReturn write_extra_joint_params(const std::vector<size_t> & indices);
  void reset_command();
  void reset_joint_command(size_t index);

  double effort_command_to_motor(size_t index) const;
  double effort_state_from_motor(size_t index, double motor_effort) const;

  static bool is_position_family(ControlMode mode);
  static bool parse_control_mode(const std::string & value, ControlMode & mode);
  static const char * mode_name(ControlMode mode);

  std::unique_ptr<DynamixelDriver> driver_;
  std::vector<Joint> joints_;
  std::string port_name_;
  int baud_rate_{0};
  bool use_dummy_{false};
  bool torque_enable_param_{true};
  bool torque_enabled_{false};
  /// Consecutive read_states() failures tolerated before read() escalates to
  /// return_type::ERROR; see read_joint_states() callers.
  int read_error_tolerance_{5};
  int consecutive_read_failures_{0};
  /// Consecutive driver write_*() failures tolerated before write() escalates
  /// to return_type::ERROR; see handle_write_result().
  int write_error_tolerance_{5};
  int consecutive_write_failures_{0};
  /// Set by read_joint_states() the first time it succeeds after activation
  /// (or cleared by on_activate()/on_deactivate()); write() stays silent
  /// until then so it never sends a command derived from the NaN/zero state
  /// init_impl() seeds every joint with (#92).
  bool has_valid_state_{false};
  /// Latched by a failed perform_command_mode_switch(): the affected joints
  /// are de-energized, so write() reports an error until the switch succeeds
  /// or the component is re-activated.
  bool switch_failed_{false};

  // Legacy write()-heuristic state (joints claiming position and velocity).
  ControlMode legacy_mode_{ControlMode::Position};
  bool legacy_warned_{false};
  bool unknown_interface_warned_{false};

  // Computed in prepare_command_mode_switch(), applied in
  // perform_command_mode_switch().
  bool pending_valid_{false};
  std::vector<ControlMode> pending_modes_;
  std::vector<bool> pending_switch_;
  std::vector<bool> pending_legacy_;
  std::vector<std::set<std::string>> pending_claims_;
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
