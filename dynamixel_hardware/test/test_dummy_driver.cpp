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

#include <gmock/gmock.h>

#include <cstdint>
#include <vector>

#include "dynamixel_hardware/dummy_driver.hpp"

namespace
{

using dynamixel_hardware::ControlMode;
using dynamixel_hardware::DummyDriver;

std::vector<uint8_t> two_ids()
{
  return {1, 2};
}

TEST(TestDummyDriver, connection_and_setup_always_succeed)
{
  DummyDriver driver;
  EXPECT_TRUE(driver.connect("/dev/ttyUSB0", 57600));
  uint16_t model_number = 42;
  EXPECT_TRUE(driver.ping(1, &model_number));
  EXPECT_TRUE(driver.ping(2));
  EXPECT_TRUE(driver.setup(two_ids()));
  EXPECT_TRUE(driver.set_torque(1, true));
  EXPECT_TRUE(driver.set_torque(1, false));
  EXPECT_TRUE(driver.set_control_mode(1, ControlMode::Velocity));
  EXPECT_TRUE(driver.write_item(1, "Profile_Velocity", 100));
  EXPECT_EQ("", driver.last_error());
  driver.disconnect();
}

TEST(TestDummyDriver, states_default_to_zero_after_setup)
{
  DummyDriver driver;
  ASSERT_TRUE(driver.setup(two_ids()));
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  ASSERT_TRUE(driver.read_states(two_ids(), positions, velocities, efforts));
  ASSERT_EQ(2u, positions.size());
  ASSERT_EQ(2u, velocities.size());
  ASSERT_EQ(2u, efforts.size());
  EXPECT_DOUBLE_EQ(0.0, positions[0]);
  EXPECT_DOUBLE_EQ(0.0, velocities[0]);
  EXPECT_DOUBLE_EQ(0.0, efforts[0]);
}

TEST(TestDummyDriver, position_commands_reflect_to_states)
{
  DummyDriver driver;
  ASSERT_TRUE(driver.setup(two_ids()));
  ASSERT_TRUE(driver.write_positions(two_ids(), {0.5, -1.25}));
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  ASSERT_TRUE(driver.read_states(two_ids(), positions, velocities, efforts));
  EXPECT_DOUBLE_EQ(0.5, positions[0]);
  EXPECT_DOUBLE_EQ(-1.25, positions[1]);
}

// Regression for
// https://github.com/dynamixel-community/dynamixel_hardware/issues/71:
// dummy velocity control previously did nothing.
TEST(TestDummyDriver, velocity_commands_integrate_position_over_tick)
{
  DummyDriver driver;
  ASSERT_TRUE(driver.setup(two_ids()));
  ASSERT_TRUE(driver.set_control_mode(1, ControlMode::Velocity));
  ASSERT_TRUE(driver.set_control_mode(2, ControlMode::Velocity));
  ASSERT_TRUE(driver.write_velocities(two_ids(), {1.0, -2.0}));
  for (int i = 0; i < 100; i++) {
    driver.tick(0.01);
  }
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  ASSERT_TRUE(driver.read_states(two_ids(), positions, velocities, efforts));
  EXPECT_NEAR(1.0, positions[0], 1e-9);
  EXPECT_NEAR(-2.0, positions[1], 1e-9);
  EXPECT_DOUBLE_EQ(1.0, velocities[0]);
  EXPECT_DOUBLE_EQ(-2.0, velocities[1]);
}

TEST(TestDummyDriver, leaving_velocity_mode_stops_integration)
{
  DummyDriver driver;
  ASSERT_TRUE(driver.setup(two_ids()));
  ASSERT_TRUE(driver.set_control_mode(1, ControlMode::Velocity));
  ASSERT_TRUE(driver.write_velocities({1}, {1.0}));
  ASSERT_TRUE(driver.set_control_mode(1, ControlMode::Position));
  driver.tick(1.0);
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  ASSERT_TRUE(driver.read_states({1}, positions, velocities, efforts));
  EXPECT_DOUBLE_EQ(0.0, positions[0]);
  EXPECT_DOUBLE_EQ(0.0, velocities[0]);
}

}  // namespace
