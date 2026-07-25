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

#ifndef DYNAMIXEL_HARDWARE__DUMMY_DRIVER_HPP_
#define DYNAMIXEL_HARDWARE__DUMMY_DRIVER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_hardware/dynamixel_driver.hpp"

namespace dynamixel_hardware
{

/// In-memory driver backing the use_dummy hardware parameter.
/// Production code (not a test double): it lets the full plugin logic run
/// without hardware, for demos and integration tests.
class DummyDriver : public DynamixelDriver
{
public:
  /// Per-servo emulated state. Only the DynamixelDriver virtual surface is
  /// contract-fixed; the control-mode rework (M3) may replace these
  /// emulation internals wholesale.
  struct ServoState
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double pwm{0.0};
    ControlMode mode{ControlMode::Position};
  };

  bool connect(const std::string & port_name, int baud_rate) override;
  void disconnect() override;
  bool ping(uint8_t id, uint16_t * model_number = nullptr) override;
  bool setup(const std::vector<uint8_t> & ids) override;
  bool set_torque(uint8_t id, bool enabled) override;
  bool set_control_mode(uint8_t id, ControlMode mode) override;
  void tick(double period_sec) override;
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

private:
  std::unordered_map<uint8_t, ServoState> servos_;
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DUMMY_DRIVER_HPP_
