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
#include <cmath>
#include <memory>
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
constexpr const char * kGoalCurrentItem = "Goal_Current";
constexpr const char * kGoalTorqueItem = "Goal_Torque";
constexpr const char * kGoalPwmItem = "Goal_PWM";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr double kGoalPwmTicksPerDuty = 885.0;
}  // namespace

bool WorkbenchDriver::connect(const std::string & port_name, int baud_rate)
{
  workbench_ = std::make_unique<DynamixelWorkbench>();
  const char * log = nullptr;
  if (!workbench_->init(port_name.c_str(), baud_rate, &log)) {
    capture_log(log);
    return false;
  }
  return true;
}

void WorkbenchDriver::disconnect()
{
  // Destroying an initialized workbench closes the serial port. A workbench that
  // was never initialized must never be destroyed at all — its base destructor
  // dereferences port/packet handler pointers that init() would have set.
  workbench_.reset();
  // getItemInfo() returns pointers owned by the (now-destroyed) workbench;
  // drop them so a stale control_items_ can never be dereferenced after
  // reconnecting.
  control_items_.clear();
  control_modes_.clear();
  model_names_.clear();
  lead_model_name_.clear();
  goal_current_index_ = -1;
  goal_pwm_index_ = -1;
  setup_done_ = false;
}

bool WorkbenchDriver::ensure_workbench()
{
  if (!workbench_) {
    last_error_ = "not connected";
    return false;
  }
  return true;
}

bool WorkbenchDriver::ensure_setup()
{
  if (!ensure_workbench()) {
    return false;
  }
  // setup() populates control_items_ before it registers the sync-write/
  // sync-read handlers, so checking control_items_ alone would report "set
  // up" even when a later handler-registration failure left some of those
  // handler indices never registered. setup_done_ is only set true after
  // the last handler registers successfully.
  if (!setup_done_) {
    last_error_ = "not set up";
    return false;
  }
  return true;
}

bool WorkbenchDriver::ensure_finite_commands(
  const std::vector<uint8_t> & ids, const std::vector<double> & values, const char * label)
{
  if (values.size() != ids.size()) {
    last_error_ = std::string(label) + " command count (" + std::to_string(values.size()) +
      ") does not match id count (" + std::to_string(ids.size()) + ")";
    return false;
  }
  for (size_t i = 0; i < values.size(); i++) {
    // Checked in the type the conversion actually uses, not in double: every
    // caller below immediately narrows with static_cast<float>(), and
    // convertRadian2Value()/convertVelocity2Value()/convertCurrent2Value()
    // assign the result to an int32_t. A finite double above ~3.4e38 becomes
    // inf as a float, and converting a non-finite float to an integer type is
    // undefined behavior -- so the magnitudes that survive the double check
    // but not the narrowing are exactly the ones this guard must catch. NaN
    // and infinities narrow to themselves and are still caught.
    if (!std::isfinite(static_cast<float>(values[i]))) {
      last_error_ = "ID " + std::to_string(ids[i]) + " received a " + std::string(label) +
        " command at index " + std::to_string(i) +
        " that is not finite in the servo's float units: " + std::to_string(values[i]);
      return false;
    }
  }
  return true;
}

bool WorkbenchDriver::ping(uint8_t id, uint16_t * model_number)
{
  if (!ensure_workbench()) {
    return false;
  }
  const char * log = nullptr;
  uint16_t model = 0;
  if (!workbench_->ping(id, &model, &log)) {
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
  if (!ensure_workbench()) {
    return false;
  }
  // A failed re-setup() must not leave stale entries (pointers into a
  // previous connection's now-freed workbench, or handler indices that no
  // longer match) sitting around for a later call to trip over.
  control_items_.clear();
  control_modes_.clear();
  model_names_.clear();
  lead_model_name_.clear();
  goal_current_index_ = -1;
  goal_pwm_index_ = -1;
  setup_done_ = false;
  if (ids.empty()) {
    last_error_ = "no joint ids configured";
    return false;
  }
  const char * log = nullptr;

  for (const auto id : ids) {
    const char * name = workbench_->getModelName(id, &log);
    model_names_[id] = name != nullptr ? name : "unknown";
    control_modes_[id] = ControlMode::Position;
  }
  // ids[0] is the lead model: setup() only probes its control table below to
  // decide which sync-write handlers to register (getItemInfo() needs a
  // concrete id and every configured servo shares the same bus protocol in
  // practice). Remember its name for "handler not available" diagnostics.
  lead_model_name_ = model_names_[ids[0]];

  // Control-table name fallbacks keep both Protocol 2.0 (X series etc.) and
  // older Protocol 1.0 servos working.
  const ControlItem * goal_position = workbench_->getItemInfo(ids[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    last_error_ = std::string("control item not found: ") + kGoalPositionItem;
    return false;
  }

  const ControlItem * goal_velocity = workbench_->getItemInfo(ids[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = workbench_->getItemInfo(ids[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    last_error_ = std::string("control item not found: ") + kGoalVelocityItem;
    return false;
  }

  const ControlItem * present_position = workbench_->getItemInfo(ids[0], kPresentPositionItem);
  if (present_position == nullptr) {
    last_error_ = std::string("control item not found: ") + kPresentPositionItem;
    return false;
  }

  const ControlItem * present_velocity = workbench_->getItemInfo(ids[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = workbench_->getItemInfo(ids[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    last_error_ = std::string("control item not found: ") + kPresentVelocityItem;
    return false;
  }

  const ControlItem * present_current = workbench_->getItemInfo(ids[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = workbench_->getItemInfo(ids[0], kPresentLoadItem);
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
  if (!workbench_->addSyncWriteHandler(
      control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
      &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_->addSyncWriteHandler(
      control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
      &log))
  {
    capture_log(log);
    return false;
  }

  // Handler indices are assigned in registration order starting at 0:
  // Goal_Position 0, Goal_Velocity/Moving_Speed 1, then Goal_Current and
  // Goal_PWM when the lead model's control table has them. The actually
  // assigned indices are stored (never hardcoded 2/3 at a call site) since a
  // bus whose lead model lacks Goal_Current shifts Goal_PWM down by one.
  int next_index = kGoalVelocityIndex + 1;
  const ControlItem * goal_current = workbench_->getItemInfo(ids[0], kGoalCurrentItem);
  if (goal_current != nullptr) {
    if (!workbench_->addSyncWriteHandler(goal_current->address, goal_current->data_length, &log)) {
      capture_log(log);
      return false;
    }
    control_items_[kGoalCurrentItem] = goal_current;
    goal_current_index_ = next_index++;
  }
  const ControlItem * goal_pwm = workbench_->getItemInfo(ids[0], kGoalPwmItem);
  if (goal_pwm != nullptr) {
    if (!workbench_->addSyncWriteHandler(goal_pwm->address, goal_pwm->data_length, &log)) {
      capture_log(log);
      return false;
    }
    control_items_[kGoalPwmItem] = goal_pwm;
    goal_pwm_index_ = next_index++;
  }

  uint16_t start_address = 0;
  uint16_t read_length = 0;
  compute_read_window(
    *control_items_[kPresentPositionItem], *control_items_[kPresentVelocityItem],
    *control_items_[kPresentCurrentItem], start_address, read_length);
  if (!workbench_->addSyncReadHandler(start_address, read_length, &log)) {
    capture_log(log);
    return false;
  }

  setup_done_ = true;
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
  if (!ensure_workbench()) {
    return false;
  }
  const char * log = nullptr;
  const bool ok = enabled ? workbench_->torqueOn(id, &log) : workbench_->torqueOff(id, &log);
  if (!ok) {
    capture_log(log);
  }
  return ok;
}

bool WorkbenchDriver::set_control_mode(uint8_t id, ControlMode mode)
{
  if (!ensure_workbench()) {
    return false;
  }

  // Capability guard: fail before ever touching the workbench setter when
  // the model's control table lacks the item the mode needs (e.g. current
  // control on AX-12, torque control outside MX-64/106).
  const char * item = required_item_for(mode);
  if (item != nullptr && workbench_->getItemInfo(id, item) == nullptr) {
    last_error_ = "ID " + std::to_string(id) + " (model " + model_name(id) +
      ") does not support the requested control mode: its control table has no '" +
      std::string(item) + "'";
    return false;
  }
  // A servo's own control table can have Goal_Current/Goal_PWM even when the
  // bus's lead model does not, since setup() only registers those sync-write
  // handlers when ids[0]'s control table has them. Reject the switch here,
  // at the point the guard is meant to fire, rather than letting it succeed
  // and have every later write_efforts()/write_pwms() call fail instead.
  if ((mode == ControlMode::Current || mode == ControlMode::CurrentBasedPosition) &&
    setup_done_ && goal_current_index_ < 0)
  {
    last_error_ = "ID " + std::to_string(id) + " (model " + model_name(id) +
      ") cannot use this control mode: no Goal_Current sync-write handler is registered " +
      "(lead model " + lead_model_name_ + " has no Goal_Current)";
    return false;
  }
  if (mode == ControlMode::PWM && setup_done_ && goal_pwm_index_ < 0) {
    last_error_ = "ID " + std::to_string(id) + " (model " + model_name(id) +
      ") cannot use this control mode: no Goal_PWM sync-write handler is registered " +
      "(lead model " + lead_model_name_ + " has no Goal_PWM)";
    return false;
  }

  const char * log = nullptr;
  bool ok = false;
  switch (mode) {
    case ControlMode::Position:
      ok = workbench_->setPositionControlMode(id, &log);
      break;
    case ControlMode::Velocity:
      ok = workbench_->setVelocityControlMode(id, &log);
      break;
    case ControlMode::Current:
      ok = workbench_->setCurrentControlMode(id, &log);
      break;
    case ControlMode::Torque:
      ok = workbench_->setTorqueControlMode(id, &log);
      break;
    case ControlMode::ExtendedPosition:
      ok = workbench_->setExtendedPositionControlMode(id, &log);
      break;
    case ControlMode::MultiTurn:
      ok = workbench_->setMultiTurnControlMode(id, &log);
      break;
    case ControlMode::CurrentBasedPosition:
      ok = workbench_->setCurrentBasedPositionControlMode(id, &log);
      break;
    case ControlMode::PWM:
      ok = workbench_->setPWMControlMode(id, &log);
      break;
  }
  if (!ok) {
    capture_log(log);
    last_error_ = "Failed to set control mode for ID " + std::to_string(id) + " (model " +
      model_name(id) + "): " + last_error_;
    return false;
  }
  control_modes_[id] = mode;
  return true;
}

bool WorkbenchDriver::write_positions(
  const std::vector<uint8_t> & ids, const std::vector<double> & radians)
{
  if (!ensure_finite_commands(ids, radians, "position")) {
    return false;
  }
  if (!ensure_setup()) {
    return false;
  }
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;  // syncWrite takes non-const pointers
  std::vector<int32_t> commands(ids.size(), 0);
  for (size_t i = 0; i < ids.size(); i++) {
    commands[i] = workbench_->convertRadian2Value(ids[i], static_cast<float>(radians[i]));
  }
  if (!workbench_->syncWrite(
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
  if (!ensure_finite_commands(ids, rad_per_sec, "velocity")) {
    return false;
  }
  if (!ensure_setup()) {
    return false;
  }
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;
  std::vector<int32_t> commands(ids.size(), 0);
  for (size_t i = 0; i < ids.size(); i++) {
    commands[i] = workbench_->convertVelocity2Value(ids[i], static_cast<float>(rad_per_sec[i]));
  }
  if (!workbench_->syncWrite(
      kGoalVelocityIndex, mutable_ids.data(), mutable_ids.size(), commands.data(), 1, &log))
  {
    capture_log(log);
    return false;
  }
  return true;
}

bool WorkbenchDriver::write_efforts(
  const std::vector<uint8_t> & ids, const std::vector<double> & values)
{
  if (!ensure_finite_commands(ids, values, "effort")) {
    return false;
  }
  if (!ensure_setup()) {
    return false;
  }
  const char * log = nullptr;
  std::vector<uint8_t> current_ids;
  std::vector<int32_t> current_commands;

  for (size_t i = 0; i < ids.size(); i++) {
    const auto it = control_modes_.find(ids[i]);
    const ControlMode mode = it != control_modes_.end() ? it->second : ControlMode::Position;
    switch (mode) {
      case ControlMode::Current:
      case ControlMode::CurrentBasedPosition:
        // Values arrive as motor-side mA (the plugin converts from Nm before
        // calling); convert to ticks and batch into the Goal_Current sync write.
        current_ids.push_back(ids[i]);
        current_commands.push_back(
          workbench_->convertCurrent2Value(ids[i], static_cast<float>(values[i])));
        break;
      case ControlMode::Torque:
        // Protocol 1.0 MX has no sync-write handler for Goal_Torque; write per id.
        if (!workbench_->itemWrite(
            ids[i], kGoalTorqueItem,
            workbench_->convertCurrent2Value(ids[i], static_cast<float>(values[i])), &log))
        {
          capture_log(log);
          last_error_ = "Goal_Torque write failed for ID " + std::to_string(ids[i]) + ": " +
            last_error_;
          return false;
        }
        break;
      default:
        last_error_ = "ID " + std::to_string(ids[i]) +
          " received an effort command but is not in a current/torque control mode";
        return false;
    }
  }

  if (!current_ids.empty()) {
    if (goal_current_index_ < 0) {
      last_error_ = "Goal_Current sync write handler is not available (lead model " +
        lead_model_name_ + " has no Goal_Current)";
      return false;
    }
    if (!workbench_->syncWrite(
        static_cast<uint8_t>(goal_current_index_), current_ids.data(), current_ids.size(),
        current_commands.data(), 1, &log))
    {
      capture_log(log);
      return false;
    }
  }
  return true;
}

bool WorkbenchDriver::write_pwms(
  const std::vector<uint8_t> & ids, const std::vector<double> & duty_ratios)
{
  if (!ensure_finite_commands(ids, duty_ratios, "PWM duty ratio")) {
    return false;
  }
  if (!ensure_setup()) {
    return false;
  }
  if (goal_pwm_index_ < 0) {
    last_error_ = "Goal_PWM sync write handler is not available (lead model " +
      lead_model_name_ + " has no Goal_PWM)";
    return false;
  }
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;  // syncWrite takes non-const pointers
  std::vector<int32_t> commands(ids.size(), 0);
  for (size_t i = 0; i < ids.size(); i++) {
    commands[i] = duty_to_pwm_ticks(duty_ratios[i]);
  }
  if (!workbench_->syncWrite(
      static_cast<uint8_t>(goal_pwm_index_), mutable_ids.data(), mutable_ids.size(),
      commands.data(), 1, &log))
  {
    capture_log(log);
    return false;
  }
  return true;
}

bool WorkbenchDriver::read_states(
  const std::vector<uint8_t> & ids, std::vector<double> & positions,
  std::vector<double> & velocities, std::vector<double> & efforts)
{
  if (!ensure_setup()) {
    return false;
  }
  const char * log = nullptr;
  std::vector<uint8_t> mutable_ids = ids;
  std::vector<int32_t> position_values(ids.size(), 0);
  std::vector<int32_t> velocity_values(ids.size(), 0);
  std::vector<int32_t> current_values(ids.size(), 0);

  if (!workbench_->syncRead(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(), &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_->getSyncReadData(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(),
      control_items_[kPresentCurrentItem]->address,
      control_items_[kPresentCurrentItem]->data_length, current_values.data(), &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_->getSyncReadData(
      kPresentPositionVelocityCurrentIndex, mutable_ids.data(), mutable_ids.size(),
      control_items_[kPresentVelocityItem]->address,
      control_items_[kPresentVelocityItem]->data_length, velocity_values.data(), &log))
  {
    capture_log(log);
    return false;
  }

  if (!workbench_->getSyncReadData(
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
    positions[i] = workbench_->convertValue2Radian(ids[i], position_values[i]);
    velocities[i] = workbench_->convertValue2Velocity(ids[i], velocity_values[i]);
    efforts[i] = workbench_->convertValue2Current(current_values[i]);
  }
  return true;
}

bool WorkbenchDriver::write_item(uint8_t id, const std::string & item, int32_t value)
{
  if (!ensure_workbench()) {
    return false;
  }
  const char * log = nullptr;
  if (!workbench_->itemWrite(id, item.c_str(), value, &log)) {
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

std::string WorkbenchDriver::model_name(uint8_t id) const
{
  const auto it = model_names_.find(id);
  return it != model_names_.end() ? it->second : "unknown";
}

int32_t WorkbenchDriver::duty_to_pwm_ticks(double duty_ratio)
{
  return static_cast<int32_t>(
    std::lround(std::clamp(duty_ratio, -1.0, 1.0) * kGoalPwmTicksPerDuty));
}

const char * WorkbenchDriver::required_item_for(ControlMode mode)
{
  switch (mode) {
    case ControlMode::Current:
    case ControlMode::CurrentBasedPosition:
      return kGoalCurrentItem;
    case ControlMode::Torque:
      return kGoalTorqueItem;
    case ControlMode::PWM:
      return kGoalPwmItem;
    default:
      // Position/Velocity exist on every model; ExtendedPosition/MultiTurn
      // support is decided by the workbench setter itself (its failure is
      // wrapped with the model name in set_control_mode()).
      return nullptr;
  }
}

}  // namespace dynamixel_hardware
