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

#include "dynamixel_hardware/dummy_driver.hpp"

#include <string>
#include <vector>

namespace dynamixel_hardware
{

bool DummyDriver::connect(const std::string & /* port_name */, int /* baud_rate */)
{
  return true;
}

void DummyDriver::disconnect()
{
}

bool DummyDriver::ping(uint8_t /* id */, uint16_t * model_number)
{
  if (model_number != nullptr) {
    *model_number = 0;
  }
  return true;
}

bool DummyDriver::setup(const std::vector<uint8_t> & ids)
{
  for (const auto id : ids) {
    servos_.emplace(id, ServoState{});
  }
  return true;
}

bool DummyDriver::set_torque(uint8_t /* id */, bool /* enabled */)
{
  return true;
}

bool DummyDriver::set_control_mode(uint8_t id, ControlMode mode)
{
  auto & servo = servos_[id];
  if (servo.mode != mode) {
    // A real servo stops when its operating mode is rewritten (torque has to
    // be cycled around the write); emulate that by zeroing the velocity.
    servo.velocity = 0.0;
    servo.mode = mode;
  }
  return true;
}

void DummyDriver::tick(double period_sec)
{
  for (auto & entry : servos_) {
    auto & servo = entry.second;
    if (servo.mode == ControlMode::Velocity) {
      servo.position += servo.velocity * period_sec;
    }
  }
}

bool DummyDriver::write_positions(
  const std::vector<uint8_t> & ids, const std::vector<double> & radians)
{
  for (size_t i = 0; i < ids.size(); i++) {
    servos_[ids[i]].position = radians[i];
  }
  return true;
}

bool DummyDriver::write_velocities(
  const std::vector<uint8_t> & ids, const std::vector<double> & rad_per_sec)
{
  for (size_t i = 0; i < ids.size(); i++) {
    servos_[ids[i]].velocity = rad_per_sec[i];
  }
  return true;
}

bool DummyDriver::write_efforts(
  const std::vector<uint8_t> & ids, const std::vector<double> & values)
{
  for (size_t i = 0; i < ids.size(); i++) {
    servos_[ids[i]].effort = values[i];
  }
  return true;
}

bool DummyDriver::write_pwms(
  const std::vector<uint8_t> & ids, const std::vector<double> & duty_ratios)
{
  for (size_t i = 0; i < ids.size(); i++) {
    servos_[ids[i]].pwm = duty_ratios[i];
  }
  return true;
}

bool DummyDriver::read_states(
  const std::vector<uint8_t> & ids, std::vector<double> & positions,
  std::vector<double> & velocities, std::vector<double> & efforts)
{
  positions.resize(ids.size());
  velocities.resize(ids.size());
  efforts.resize(ids.size());
  for (size_t i = 0; i < ids.size(); i++) {
    const auto & servo = servos_[ids[i]];
    positions[i] = servo.position;
    velocities[i] = servo.velocity;
    efforts[i] = servo.effort;
  }
  return true;
}

bool DummyDriver::write_item(uint8_t /* id */, const std::string & /* item */, int32_t /* value */)
{
  return true;
}

std::string DummyDriver::last_error() const
{
  return "";
}

}  // namespace dynamixel_hardware
