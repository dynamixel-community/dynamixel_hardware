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
#include <vector>

#include "dynamixel_hardware/dynamixel_driver.hpp"

namespace dynamixel_hardware
{

/// Production driver: all DynamixelWorkbench (serial) I/O lives here.
///
/// Note: dynamixel_workbench_toolbox also defines a class named
/// DynamixelDriver in the global namespace; inside this namespace the
/// unqualified name always refers to dynamixel_hardware::DynamixelDriver.
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

private:
  void capture_log(const char * log);
  /// True when connect() has allocated the workbench; otherwise sets last_error_.
  bool ensure_workbench();

  std::unique_ptr<DynamixelWorkbench> workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::string last_error_;
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__WORKBENCH_DRIVER_HPP_
