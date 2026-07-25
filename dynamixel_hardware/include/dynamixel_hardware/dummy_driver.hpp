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

/// In-memory driver backing the `use_dummy` hardware parameter.
///
/// Emulates every ControlMode so that mode switching, launch tests and demos
/// behave equivalently with and without hardware (design spec section 5.5):
///   Position                -> state.position <- cmd (clamped to [-pi, pi]),
///                              state.velocity <- (cmd - previous) / period
///   ExtendedPosition,
///   MultiTurn               -> same as Position but without the clamp
///   Velocity                -> position += cmd * period (integrated in tick()),
///                              velocity <- cmd (regression #71)
///   Current, Torque         -> state.effort <- cmd, kinematic state held
///   CurrentBasedPosition    -> position tracked (no clamp), state.effort <- cap
///   PWM                     -> duty accepted, state.effort mirrors the duty
///
/// Writes that do not match an id's active mode return false with last_error
/// set, so plugin dispatch bugs fail loudly in tests.
class DummyDriver : public DynamixelDriver
{
public:
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
  struct Servo
  {
    ControlMode mode{ControlMode::Position};
    bool torque{false};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double goal_velocity{0.0};
  };

  Servo * find(uint8_t id);

  std::unordered_map<uint8_t, Servo> servos_;
  double last_period_{0.0};
  bool connected_{false};
  std::string last_error_;
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DUMMY_DRIVER_HPP_
