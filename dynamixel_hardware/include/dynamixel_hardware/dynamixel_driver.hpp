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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_DRIVER_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace dynamixel_hardware
{

enum class ControlMode
{
  Position,
  Velocity,
  Current,
  Torque,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

/// Abstract boundary between the ros2_control plugin and the servo protocol.
/// Implementations: WorkbenchDriver (serial I/O via DynamixelWorkbench),
/// DummyDriver (in-memory emulation for use_dummy), MockDriver (tests).
class DynamixelDriver
{
public:
  virtual ~DynamixelDriver() = default;

  virtual bool connect(const std::string & port_name, int baud_rate) = 0;
  virtual void disconnect() = 0;
  virtual bool ping(uint8_t id, uint16_t * model_number = nullptr) = 0;
  /// Resolve control-table items and register sync read/write handlers.
  virtual bool setup(const std::vector<uint8_t> & ids) = 0;
  virtual bool set_torque(uint8_t id, bool enabled) = 0;
  virtual bool set_control_mode(uint8_t id, ControlMode mode) = 0;
  /// Called once at the top of every write() cycle; DummyDriver integrates here.
  virtual void tick(double period_sec) {(void)period_sec;}
  virtual bool write_positions(
    const std::vector<uint8_t> & ids, const std::vector<double> & radians) = 0;
  virtual bool write_velocities(
    const std::vector<uint8_t> & ids, const std::vector<double> & rad_per_sec) = 0;
  virtual bool write_efforts(
    const std::vector<uint8_t> & ids, const std::vector<double> & values) = 0;
  virtual bool write_pwms(
    const std::vector<uint8_t> & ids, const std::vector<double> & duty_ratios) = 0;
  /// On success, implementations must resize positions, velocities and
  /// efforts to ids.size() before returning true -- the plugin indexes all
  /// three by joint index without checking their size.
  virtual bool read_states(
    const std::vector<uint8_t> & ids, std::vector<double> & positions,
    std::vector<double> & velocities, std::vector<double> & efforts) = 0;
  virtual bool write_item(uint8_t id, const std::string & item, int32_t value) = 0;
  virtual std::string last_error() const = 0;
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_DRIVER_HPP_
