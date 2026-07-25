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

#ifndef MOCK_DRIVER_HPP_
#define MOCK_DRIVER_HPP_

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "dynamixel_hardware/dynamixel_driver.hpp"

namespace dynamixel_hardware
{

class MockDriver : public DynamixelDriver
{
public:
  MOCK_METHOD(bool, connect, (const std::string & port_name, int baud_rate), (override));
  MOCK_METHOD(void, disconnect, (), (override));
  MOCK_METHOD(bool, ping, (uint8_t id, uint16_t * model_number), (override));
  MOCK_METHOD(bool, setup, (const std::vector<uint8_t> & ids), (override));
  MOCK_METHOD(bool, set_torque, (uint8_t id, bool enabled), (override));
  MOCK_METHOD(bool, set_control_mode, (uint8_t id, ControlMode mode), (override));
  MOCK_METHOD(void, tick, (double period_sec), (override));
  MOCK_METHOD(
    bool, write_positions,
    (const std::vector<uint8_t> & ids, const std::vector<double> & radians), (override));
  MOCK_METHOD(
    bool, write_velocities,
    (const std::vector<uint8_t> & ids, const std::vector<double> & rad_per_sec), (override));
  MOCK_METHOD(
    bool, write_efforts,
    (const std::vector<uint8_t> & ids, const std::vector<double> & values), (override));
  MOCK_METHOD(
    bool, write_pwms,
    (const std::vector<uint8_t> & ids, const std::vector<double> & duty_ratios), (override));
  MOCK_METHOD(
    bool, read_states,
    (const std::vector<uint8_t> & ids, std::vector<double> & positions,
    std::vector<double> & velocities, std::vector<double> & efforts),
    (override));
  MOCK_METHOD(bool, write_item, (uint8_t id, const std::string & item, int32_t value), (override));
  MOCK_METHOD(std::string, last_error, (), (const, override));
};

}  // namespace dynamixel_hardware

#endif  // MOCK_DRIVER_HPP_
