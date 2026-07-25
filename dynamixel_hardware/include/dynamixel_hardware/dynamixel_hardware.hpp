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
#include <unordered_map>
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

/// Largest valid individual Dynamixel servo id. Per
/// dynamixel_sdk/packet_handler.h (Protocol 2.0), MAX_ID is 0xFC (252); id
/// 253 (0xFD) is unused and 254 (0xFE, BROADCAST_ID) addresses every servo at
/// once, so neither can name one physical joint.
constexpr int kMaxDynamixelId = 252;

/// Which finite values parse_double_param() accepts, beyond "must parse and
/// be finite" (always required). The three per-joint parameters routed
/// through it each need a different rule: offset allows zero and negatives,
/// gear_ratio allows negatives (direction inversion) but not zero (divide by
/// zero), torque_constant allows neither.
enum class DoubleParamRule
{
  kFinite,          ///< Any finite value, including zero and negatives (offset).
  kFiniteNonZero,   ///< Finite and non-zero; negatives allowed (gear_ratio).
  kFinitePositive,  ///< Finite and strictly positive (torque_constant).
};

/// Which of the extra control-table parameters write_extra_joint_params()
/// writes. A mode change resets the RAM registers among them to their
/// defaults, which is the whole reason they are rewritten after every switch;
/// the EEPROM ones survive it and only need writing once.
enum class ExtraParamScope
{
  kAll,      ///< Every configured parameter (on_configure).
  kRamOnly,  ///< Only the ones a mode change resets (apply_mode_switch).
};

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
  /// Whether this servo is energized right now. Tracked per joint because a
  /// partial failure -- one servo torqued, the next one refusing -- is a state
  /// no hardware-wide flag can represent. Invariant governing every site that
  /// assigns it: `true` means a driver call CONFIRMED torque on and nothing
  /// has tried to turn it off since. So it is set only after a successful
  /// torque-on, and cleared before a torque-off is attempted; a rejected
  /// torque-on leaves it false. Reading false therefore means "not known to be
  /// energized", which is why the mode switch de-energizes such a joint anyway
  /// but never re-energizes it.
  bool torque_enabled{false};
  /// Nm/A; 0.0 means unset and the effort interfaces carry milliamps.
  double torque_constant{0.0};
  /// Motor revolutions per joint revolution. Positions/velocities are divided
  /// by it and efforts multiplied by it going from motor side to joint side;
  /// commands are the inverse. Negative inverts the direction; zero is
  /// rejected at init.
  double gear_ratio{1.0};
  /// Joint-side position offset: subtracted after the gear conversion on
  /// read, added back before it on write. Parsed from the 'offset' per-joint
  /// parameter (#93); 0.0 (the default) is a no-op.
  double offset{0.0};
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
  /// Parses hardware_parameters[name] as an int into out, validating out >=
  /// min_value. Absent key: returns SUCCESS without touching out (callers
  /// decide whether the parameter is required). Non-numeric value or a
  /// parsed value below min_value: logs and returns ERROR. joint_name, when
  /// non-null, names the owning joint in the error message -- pass it for
  /// per-joint parameters and leave it null for hardware-level ones (e.g.
  /// baud_rate) so a typo'd per-joint param can be traced on a multi-joint
  /// robot.
  CallbackReturn parse_int_param(
    const std::unordered_map<std::string, std::string> & params, const char * name, int & out,
    int min_value, const char * joint_name = nullptr);
  /// Parses params[name] as a double into out, always requiring a finite
  /// value, plus whatever rule further restricts it (see DoubleParamRule).
  /// Absent key: returns SUCCESS without touching out (callers decide
  /// whether the parameter is required). Non-numeric value, non-finite
  /// value, or a value the rule rejects: logs and returns ERROR. joint_name,
  /// when non-null, names the owning joint in the error message -- pass it
  /// for per-joint parameters and leave it null for hardware-level ones
  /// (e.g. baud_rate) so a typo'd per-joint param can be traced on a
  /// multi-joint robot.
  CallbackReturn parse_double_param(
    const std::unordered_map<std::string, std::string> & params, const char * name, double & out,
    DoubleParamRule rule, const char * joint_name = nullptr);

  rclcpp::Logger logger() const;

  std::vector<uint8_t> all_ids() const;
  int find_joint(const std::string & joint_name) const;

  bool read_joint_states();
  return_type handle_write_result(const bool ok);
  return_type set_torque_all(const bool enabled);
  /// True iff every joint reports torque on. Vacuously true without joints:
  /// there is nothing de-energized to report.
  bool all_torque_enabled() const;
  /// Torque off -> set_control_mode -> extra-parameter rewrite -> torque on.
  return_type apply_mode_switch(
    const std::vector<size_t> & indices, const std::vector<ControlMode> & modes);
  /// Calls apply_mode_switch() and, on failure, latches switch_failed_ and logs
  /// the operator-facing recovery message once. This is the single call site
  /// perform_command_mode_switch() and update_legacy_heuristic() both route
  /// through, so a failed switch is latched identically regardless of which
  /// path triggered it -- previously the legacy path reported
  /// return_type::ERROR for one cycle without latching, so write() silently
  /// retried the switch every cycle after (#112 follow-up). Clearing
  /// switch_failed_ on success stays perform_command_mode_switch()'s job alone
  /// (its all_torque_enabled() / torque_enable_param_ check); this helper never
  /// clears the latch.
  return_type apply_mode_switch_or_latch(
    const std::vector<size_t> & indices, const std::vector<ControlMode> & modes);
  return_type update_legacy_heuristic();
  /// Writes the configured extra control-table parameters of every joint in
  /// indices; scope decides whether the EEPROM-resident ones are included
  /// (see ExtraParamScope).
  CallbackReturn write_extra_joint_params(
    const std::vector<size_t> & indices, ExtraParamScope scope);
  void reset_command();
  void reset_joint_command(size_t index);

  double effort_command_to_motor(size_t index) const;
  double effort_state_from_motor(size_t index, double motor_effort) const;

  /// gear_ratio/offset conversions at the driver boundary (#95/#94). Contract
  /// (fixed): gear_ratio = motor revolutions per joint revolution;
  /// joint_position = motor_position / gear_ratio, joint_velocity =
  /// motor_velocity / gear_ratio, joint_effort = motor_effort * gear_ratio;
  /// commands are the inverse. offset applies to position only:
  /// joint_reported = raw_joint_position - offset (raw = after gear
  /// conversion); commands add it back before scaling.
  double to_joint_position(size_t index, double motor_position) const;
  double to_motor_position(size_t index, double joint_position) const;
  double to_joint_velocity(size_t index, double motor_velocity) const;
  double to_motor_velocity(size_t index, double joint_velocity) const;
  double to_joint_effort(size_t index, double motor_effort) const;
  double to_motor_effort(size_t index, double joint_effort) const;

  static bool is_position_family(ControlMode mode);
  static bool parse_control_mode(const std::string & value, ControlMode & mode);
  static const char * mode_name(ControlMode mode);

  std::unique_ptr<DynamixelDriver> driver_;
  std::vector<Joint> joints_;
  std::string port_name_;
  int baud_rate_{0};
  bool use_dummy_{false};
  bool torque_enable_param_{true};
  /// Consecutive read_states() failures tolerated before read() escalates to
  /// return_type::ERROR; see read_joint_states() callers. 0 disables the
  /// escalation entirely (failures are still counted and warned about), which
  /// is the documented opt-out back to the pre-#88 behavior for a bus too
  /// noisy to survive it.
  int read_error_tolerance_{5};
  int consecutive_read_failures_{0};
  /// Consecutive driver write_*() failures tolerated before write() escalates
  /// to return_type::ERROR; see handle_write_result(). 0 disables the
  /// escalation, as on the read side.
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
