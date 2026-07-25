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

#ifndef DYNAMIXEL_HARDWARE__WORKBENCH_DRIVER_HPP_
#define DYNAMIXEL_HARDWARE__WORKBENCH_DRIVER_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_hardware/dynamixel_driver.hpp"

namespace dynamixel_hardware
{

/// Production driver: all DynamixelWorkbench (serial) I/O lives here.
///
/// Note: dynamixel_workbench_toolbox also defines a class named
/// DynamixelDriver in the global namespace; inside this namespace the
/// unqualified name always refers to dynamixel_hardware::DynamixelDriver.
///
/// set_control_mode() maps every ControlMode to its DynamixelWorkbench
/// setter and keeps a per-id active-mode map (control_modes_); setup()
/// records each id's model name (model_names_). A capability guard rejects
/// modes the model's control table cannot support with a last_error naming
/// the id and model.
class WorkbenchDriver : public DynamixelDriver
{
public:
  bool connect(const std::string & port_name, int baud_rate) override;
  void disconnect() override;
  bool ping(uint8_t id, uint16_t * model_number = nullptr) override;
  bool setup(const std::vector<uint8_t> & ids) override;
  bool set_torque(uint8_t id, bool enabled) override;
  bool set_control_mode(uint8_t id, ControlMode mode) override;
  bool write_positions(
    const std::vector<uint8_t> & ids, const std::vector<double> & radians) override;
  bool write_velocities(
    const std::vector<uint8_t> & ids, const std::vector<double> & rad_per_sec) override;
  bool write_efforts(
    const std::vector<uint8_t> & ids, const std::vector<double> & values) override;
  bool write_pwms(
    const std::vector<uint8_t> & ids, const std::vector<double> & duty_ratios) override;
  bool read_states(
    const std::vector<uint8_t> & ids, std::vector<double> & positions,
    std::vector<double> & velocities, std::vector<double> & efforts) override;
  bool write_item(uint8_t id, const std::string & item, int32_t value) override;
  std::string last_error() const override;

  /// Pure helper for the sync-read window: one bulk read spanning present
  /// current/load, velocity and position. Public and static so the unit
  /// test can drive it without serial hardware; setup() calls it when
  /// registering the sync read handler.
  static void compute_read_window(
    const ControlItem & position, const ControlItem & velocity, const ControlItem & current,
    uint16_t & start_address, uint16_t & read_length);

  /// Pure helpers, unit-tested without serial I/O.
  /// Goal_PWM is +-885 ticks for +-100 % duty (0.113 %/tick, X-series).
  /// duty_ratio must be finite -- write_pwms() guards this before calling.
  static int32_t duty_to_pwm_ticks(double duty_ratio);
  /// Control-table item a model must have to enter the mode; nullptr when the
  /// mode needs no capability check beyond the workbench setter itself.
  static const char * required_item_for(ControlMode mode);

private:
  void capture_log(const char * log);
  /// True when connect() has allocated the workbench; otherwise sets last_error_.
  bool ensure_workbench();
  /// True when the workbench is connected and setup() has completed --
  /// registered every sync handler it attempted, not merely populated
  /// control_items_ (a handler-registration failure can happen after that);
  /// otherwise sets last_error_. Calls ensure_workbench() first, so a
  /// never-connected driver still reports "not connected".
  bool ensure_setup();
  /// True when values.size() == ids.size() and every element is still finite
  /// after the narrowing to float that every conversion below performs (so
  /// magnitudes above ~3.4e38 are refused alongside NaN and the infinities);
  /// otherwise sets last_error_ naming the offending id, its index in the
  /// batch, and label (e.g. "position"), and returns false. Called first in
  /// every write_* method, before ensure_setup(), so a non-finite command --
  /// always a caller bug -- is reported precisely regardless of connection
  /// state, and the whole batch is refused rather than partially written.
  bool ensure_finite_commands(
    const std::vector<uint8_t> & ids, const std::vector<double> & values, const char * label);
  std::string model_name(uint8_t id) const;

  std::unique_ptr<DynamixelWorkbench> workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::unordered_map<uint8_t, ControlMode> control_modes_;
  std::unordered_map<uint8_t, std::string> model_names_;
  // Model name of ids[0] as setup() saw it: the servo whose control table
  // decides which of the Goal_Current/Goal_PWM sync-write handlers below get
  // registered. Named explicitly in "handler not available" diagnostics so
  // they always point at the servo actually responsible, even when the id
  // that triggered the failure is a different (fully capable) model on the
  // same bus.
  std::string lead_model_name_;
  // Actually assigned Goal_Current / Goal_PWM sync-write handler indices
  // (nominally 2 and 3; -1 when the lead model's control table lacks the
  // item -- see setup()).
  int goal_current_index_{-1};
  int goal_pwm_index_{-1};
  // Set only after setup() registers every handler it attempted; cleared at
  // the top of setup() and by disconnect(). Closes the gap where
  // control_items_ is populated before handler registration can still fail.
  bool setup_done_{false};
  std::string last_error_;
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__WORKBENCH_DRIVER_HPP_
