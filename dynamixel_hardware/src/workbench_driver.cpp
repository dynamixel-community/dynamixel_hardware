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

#include "dynamixel_hardware/workbench_driver.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace dynamixel_hardware
{

namespace
{
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
}  // namespace

bool WorkbenchDriver::connect(const std::string & port_name, int baud_rate)
{
  const char * log = nullptr;
  if (!workbench_.init(port_name.c_str(), baud_rate, &log)) {
    capture_log(log);
    return false;
  }
  return true;
}

void WorkbenchDriver::disconnect()
{
  // DynamixelWorkbench does not expose a close API; the serial port is
  // released when this object is destroyed.
}

bool WorkbenchDriver::ping(uint8_t id, uint16_t * model_number)
{
  const char * log = nullptr;
  uint16_t model = 0;
  if (!workbench_.ping(id, &model, &log)) {
    capture_log(log);
    return false;
  }
  if (model_number != nullptr) {
    *model_number = model;
  }
  return true;
}

bool WorkbenchDriver::setup(const std::vector<uint8_t> & ids)
{
  if (ids.empty()) {
    last_error_ = "no joint ids configured";
    return false;
  }
  const char * log = nullptr;

  // Control-table name fallbacks keep both Protocol 2.0 (X series etc.) and
  // older Protocol 1.0 servos working.
  const ControlItem * goal_position = workbench_.getItemInfo(ids[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    last_error_ = std::string("control item not found: ") + kGoalPositionItem;
    return false;
  }

  const ControlItem * goal_velocity = workbench_.getItemInfo(ids[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = workbench_.getItemInfo(ids[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    last_error_ = std::string("control item not found: ") + kGoalVelocityItem;
    return false;
  }

  const ControlItem * present_position = workbench_.getItemInfo(ids[0], kPresentPositionItem);
  if (present_position == nullptr) {
    last_error_ = std::string("control item not found: ") + kPresentPositionItem;
    return false;
  }

  const ControlItem * present_velocity = workbench_.getItemInfo(ids[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = workbench_.getItemInfo(ids[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    last_error_ = std::string("control item not found: ") + kPresentVelocityItem;
    return false;
  }

  const ControlItem * present_current = workbench_.getItemInfo(ids[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = workbench_.getItemInfo(ids[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    last_error_ = std::string("control item not found: ") + kPresentCurrentItem;
    return false;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  // Sync write handler indices are fixed: 0 = goal position, 1 = goal velocity.
  if (!workbench_.addSyncWriteHandler(
      control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
      &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_.addSyncWriteHandler(
      control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
      &log))
  {
    capture_log(log);
    return false;
  }

  uint16_t start_address = 0;
  uint16_t read_length = 0;
  compute_read_window(
    *control_items_[kPresentPositionItem], *control_items_[kPresentVelocityItem],
    *control_items_[kPresentCurrentItem], start_address, read_length);
  if (!workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    capture_log(log);
    return false;
  }

  return true;
}

void WorkbenchDriver::compute_read_window(
  const ControlItem & position, const ControlItem & velocity, const ControlItem & current,
  uint16_t & start_address, uint16_t & read_length)
{
  // One bulk read spans present current/load, velocity and position. The
  // historical "+2" widens the window by two bytes to bridge the address
  // gap between the current/load block and the velocity/position block on
  // the supported control tables. Kept verbatim from the pre-refactor
  // implementation.
  start_address = std::min(position.address, current.address);
  read_length = position.data_length + velocity.data_length + current.data_length + 2;
}

bool WorkbenchDriver::set_torque(uint8_t id, bool enabled)
{
  const char * log = nullptr;
  const bool ok = enabled ? workbench_.torqueOn(id, &log) : workbench_.torqueOff(id, &log);
  if (!ok) {
    capture_log(log);
  }
  return ok;
}

bool WorkbenchDriver::set_control_mode(uint8_t id, ControlMode mode)
{
  const char * log = nullptr;
  switch (mode) {
    case ControlMode::Position:
      if (!workbench_.setPositionControlMode(id, &log)) {
        capture_log(log);
        return false;
      }
      return true;
    case ControlMode::Velocity:
      if (!workbench_.setVelocityControlMode(id, &log)) {
        capture_log(log);
        return false;
      }
      return true;
    default:
      last_error_ = "mode not implemented until control-mode rework";
      return false;
  }
}

bool WorkbenchDriver::write_positions(
  const std::vector<uint8_t> & ids, const std::vector<double> & radians)
{
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;  // syncWrite takes non-const pointers
  std::vector<int32_t> commands(ids.size(), 0);
  for (size_t i = 0; i < ids.size(); i++) {
    commands[i] = workbench_.convertRadian2Value(ids[i], static_cast<float>(radians[i]));
  }
  if (!workbench_.syncWrite(
      kGoalPositionIndex, mutable_ids.data(), mutable_ids.size(), commands.data(), 1, &log))
  {
    capture_log(log);
    return false;
  }
  return true;
}

bool WorkbenchDriver::write_velocities(
  const std::vector<uint8_t> & ids, const std::vector<double> & rad_per_sec)
{
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;
  std::vector<int32_t> commands(ids.size(), 0);
  for (size_t i = 0; i < ids.size(); i++) {
    commands[i] = workbench_.convertVelocity2Value(ids[i], static_cast<float>(rad_per_sec[i]));
  }
  if (!workbench_.syncWrite(
      kGoalVelocityIndex, mutable_ids.data(), mutable_ids.size(), commands.data(), 1, &log))
  {
    capture_log(log);
    return false;
  }
  return true;
}

bool WorkbenchDriver::write_efforts(
  const std::vector<uint8_t> & /* ids */, const std::vector<double> & /* values */)
{
  last_error_ = "mode not implemented until control-mode rework";
  return false;
}

bool WorkbenchDriver::write_pwms(
  const std::vector<uint8_t> & /* ids */, const std::vector<double> & /* duty_ratios */)
{
  last_error_ = "mode not implemented until control-mode rework";
  return false;
}

bool WorkbenchDriver::read_states(
  const std::vector<uint8_t> & ids, std::vector<double> & positions,
  std::vector<double> & velocities, std::vector<double> & efforts)
{
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;
  std::vector<int32_t> position_values(ids.size(), 0);
  std::vector<int32_t> velocity_values(ids.size(), 0);
  std::vector<int32_t> current_values(ids.size(), 0);

  if (!workbench_.syncRead(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(), &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(),
      control_items_[kPresentCurrentItem]->address,
      control_items_[kPresentCurrentItem]->data_length, current_values.data(), &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(),
      control_items_[kPresentVelocityItem]->address,
      control_items_[kPresentVelocityItem]->data_length, velocity_values.data(), &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_.getSyncReadData(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(),
      control_items_[kPresentPositionItem]->address,
      control_items_[kPresentPositionItem]->data_length, position_values.data(), &log))
  {
    capture_log(log);
    return false;
  }

  positions.resize(ids.size());
  velocities.resize(ids.size());
  efforts.resize(ids.size());
  for (size_t i = 0; i < ids.size(); i++) {
    positions[i] = workbench_.convertValue2Radian(ids[i], position_values[i]);
    velocities[i] = workbench_.convertValue2Velocity(ids[i], velocity_values[i]);
    efforts[i] = workbench_.convertValue2Current(current_values[i]);
  }
  return true;
}

bool WorkbenchDriver::write_item(uint8_t id, const std::string & item, int32_t value)
{
  const char * log = nullptr;
  if (!workbench_.itemWrite(id, item.c_str(), value, &log)) {
    capture_log(log);
    return false;
  }
  return true;
}

std::string WorkbenchDriver::last_error() const
{
  return last_error_;
}

void WorkbenchDriver::capture_log(const char * log)
{
  last_error_ = (log != nullptr) ? log : "unknown DynamixelWorkbench error";
}

}  // namespace dynamixel_hardware
