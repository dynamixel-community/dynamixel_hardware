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

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace dynamixel_hardware
{

namespace
{
constexpr uint16_t kDummyModelNumber = 1030;  // XM430-W350

bool is_position_write_mode(ControlMode mode)
{
  return mode == ControlMode::Position || mode == ControlMode::ExtendedPosition ||
         mode == ControlMode::MultiTurn || mode == ControlMode::CurrentBasedPosition;
}

bool is_effort_write_mode(ControlMode mode)
{
  return mode == ControlMode::Current || mode == ControlMode::Torque ||
         mode == ControlMode::CurrentBasedPosition;
}
}  // namespace

bool DummyDriver::connect(const std::string & /* port_name */, int /* baud_rate */)
{
  connected_ = true;
  return true;
}

void DummyDriver::disconnect()
{
  connected_ = false;
}

bool DummyDriver::ping(uint8_t /* id */, uint16_t * model_number)
{
  if (model_number != nullptr) {
    *model_number = kDummyModelNumber;
  }
  return true;
}

bool DummyDriver::setup(const std::vector<uint8_t> & ids)
{
  for (const auto id : ids) {
    servos_.emplace(id, Servo{});
  }
  return true;
}

bool DummyDriver::set_torque(uint8_t id, bool enabled)
{
  auto * servo = find(id);
  if (servo == nullptr) {
    return false;
  }
  servo->torque = enabled;
  return true;
}

bool DummyDriver::set_control_mode(uint8_t id, ControlMode mode)
{
  auto * servo = find(id);
  if (servo == nullptr) {
    return false;
  }
  servo->mode = mode;
  // A real servo is torque-cycled and stopped around a mode change.
  servo->goal_velocity = 0.0;
  return true;
}

void DummyDriver::tick(double period_sec)
{
  last_period_ = period_sec;
  if (period_sec <= 0.0) {
    return;
  }
  for (auto & pair : servos_) {
    auto & servo = pair.second;
    if (servo.mode == ControlMode::Velocity) {
      servo.position += servo.goal_velocity * period_sec;
      servo.velocity = servo.goal_velocity;
    }
  }
}

bool DummyDriver::write_positions(
  const std::vector<uint8_t> & ids, const std::vector<double> & radians)
{
  for (size_t i = 0; i < ids.size(); i++) {
    auto * servo = find(ids[i]);
    if (servo == nullptr) {
      return false;
    }
    if (!is_position_write_mode(servo->mode)) {
      last_error_ = "ID " + std::to_string(ids[i]) + " is not in a position control mode";
      return false;
    }
    double target = radians[i];
    if (servo->mode == ControlMode::Position) {
      target = std::clamp(target, -M_PI, M_PI);
    }
    servo->velocity =
      last_period_ > 0.0 ? (target - servo->position) / last_period_ : 0.0;
    servo->position = target;
  }
  return true;
}

bool DummyDriver::write_velocities(
  const std::vector<uint8_t> & ids, const std::vector<double> & rad_per_sec)
{
  for (size_t i = 0; i < ids.size(); i++) {
    auto * servo = find(ids[i]);
    if (servo == nullptr) {
      return false;
    }
    if (servo->mode != ControlMode::Velocity) {
      last_error_ = "ID " + std::to_string(ids[i]) + " is not in velocity control mode";
      return false;
    }
    servo->goal_velocity = rad_per_sec[i];
  }
  return true;
}

bool DummyDriver::write_efforts(
  const std::vector<uint8_t> & ids, const std::vector<double> & values)
{
  for (size_t i = 0; i < ids.size(); i++) {
    auto * servo = find(ids[i]);
    if (servo == nullptr) {
      return false;
    }
    if (!is_effort_write_mode(servo->mode)) {
      last_error_ = "ID " + std::to_string(ids[i]) + " is not in a current/torque control mode";
      return false;
    }
    // Current/Torque: commanded effort; CurrentBasedPosition: the current cap.
    servo->effort = values[i];
  }
  return true;
}

bool DummyDriver::write_pwms(
  const std::vector<uint8_t> & ids, const std::vector<double> & duty_ratios)
{
  for (size_t i = 0; i < ids.size(); i++) {
    auto * servo = find(ids[i]);
    if (servo == nullptr) {
      return false;
    }
    if (servo->mode != ControlMode::PWM) {
      last_error_ = "ID " + std::to_string(ids[i]) + " is not in PWM control mode";
      return false;
    }
    servo->effort = duty_ratios[i];  // state.effort mirrors the duty ratio
  }
  return true;
}

bool DummyDriver::read_states(
  const std::vector<uint8_t> & ids, std::vector<double> & positions,
  std::vector<double> & velocities, std::vector<double> & efforts)
{
  positions.assign(ids.size(), 0.0);
  velocities.assign(ids.size(), 0.0);
  efforts.assign(ids.size(), 0.0);
  for (size_t i = 0; i < ids.size(); i++) {
    auto * servo = find(ids[i]);
    if (servo == nullptr) {
      return false;
    }
    positions[i] = servo->position;
    velocities[i] = servo->velocity;
    efforts[i] = servo->effort;
  }
  return true;
}

bool DummyDriver::write_item(uint8_t id, const std::string & /* item */, int32_t /* value */)
{
  return find(id) != nullptr;
}

std::string DummyDriver::last_error() const
{
  return last_error_;
}

DummyDriver::Servo * DummyDriver::find(uint8_t id)
{
  auto it = servos_.find(id);
  if (it == servos_.end()) {
    last_error_ = "Unknown Dynamixel ID " + std::to_string(id);
    return nullptr;
  }
  return &it->second;
}

}  // namespace dynamixel_hardware
