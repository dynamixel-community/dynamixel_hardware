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

#include <cmath>
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

// ---------------------------------------------------------------------------
// M3 (feat/control-modes): per-mode emulation equivalence — one test per row
// of the design-spec mode table, plus the #71 velocity-integration regression.
// ---------------------------------------------------------------------------

class DummyAllModesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(driver_.connect("/dev/null", 1000000));
    ASSERT_TRUE(driver_.setup({1, 2}));
  }

  double position_of(uint8_t id)
  {
    read();
    return positions_[id == 1 ? 0 : 1];
  }

  double velocity_of(uint8_t id)
  {
    read();
    return velocities_[id == 1 ? 0 : 1];
  }

  double effort_of(uint8_t id)
  {
    read();
    return efforts_[id == 1 ? 0 : 1];
  }

  void read()
  {
    ASSERT_TRUE(driver_.read_states({1, 2}, positions_, velocities_, efforts_));
  }

  dynamixel_hardware::DummyDriver driver_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
};

TEST_F(DummyAllModesTest, AcceptsAllEightControlModes)
{
  const dynamixel_hardware::ControlMode modes[] = {
    dynamixel_hardware::ControlMode::Position,
    dynamixel_hardware::ControlMode::Velocity,
    dynamixel_hardware::ControlMode::Current,
    dynamixel_hardware::ControlMode::Torque,
    dynamixel_hardware::ControlMode::ExtendedPosition,
    dynamixel_hardware::ControlMode::MultiTurn,
    dynamixel_hardware::ControlMode::CurrentBasedPosition,
    dynamixel_hardware::ControlMode::PWM,
  };
  for (const auto mode : modes) {
    EXPECT_TRUE(driver_.set_control_mode(1, mode));
  }
}

TEST_F(DummyAllModesTest, PingReportsDummyModel)
{
  uint16_t model_number = 0;
  EXPECT_TRUE(driver_.ping(1, &model_number));
  EXPECT_EQ(1030, model_number);  // XM430-W350
}

TEST_F(DummyAllModesTest, PositionModeReflectsCommandAndDerivesVelocity)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Position));
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_positions({1}, {0.5}));
  EXPECT_DOUBLE_EQ(0.5, position_of(1));
  EXPECT_NEAR(5.0, velocity_of(1), 1e-9);  // (0.5 - 0.0) / 0.1
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_positions({1}, {0.5}));  // unchanged command
  EXPECT_NEAR(0.0, velocity_of(1), 1e-9);
}

TEST_F(DummyAllModesTest, PositionModeClampsToSingleTurnRange)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Position));
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_positions({1}, {4.0}));
  EXPECT_DOUBLE_EQ(M_PI, position_of(1));
  ASSERT_TRUE(driver_.write_positions({1}, {-4.0}));
  EXPECT_DOUBLE_EQ(-M_PI, position_of(1));
}

TEST_F(DummyAllModesTest, ExtendedPositionAndMultiTurnDoNotWrap)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::ExtendedPosition));
  ASSERT_TRUE(driver_.set_control_mode(2, dynamixel_hardware::ControlMode::MultiTurn));
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_positions({1, 2}, {4.0 * M_PI, -12.0}));
  EXPECT_DOUBLE_EQ(4.0 * M_PI, position_of(1));
  EXPECT_DOUBLE_EQ(-12.0, position_of(2));
}

TEST_F(DummyAllModesTest, VelocityModeIntegratesOverTicks)
{
  // Regression for #71: dummy velocity control previously did nothing.
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Velocity));
  ASSERT_TRUE(driver_.write_velocities({1}, {1.0}));
  for (int i = 0; i < 5; ++i) {
    driver_.tick(0.1);
  }
  EXPECT_NEAR(0.5, position_of(1), 1e-9);
  EXPECT_DOUBLE_EQ(1.0, velocity_of(1));
}

TEST_F(DummyAllModesTest, CurrentAndTorqueModesMirrorEffortAndHoldKinematics)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Current));
  ASSERT_TRUE(driver_.set_control_mode(2, dynamixel_hardware::ControlMode::Torque));
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_efforts({1, 2}, {123.0, -45.0}));
  EXPECT_DOUBLE_EQ(123.0, effort_of(1));
  EXPECT_DOUBLE_EQ(-45.0, effort_of(2));
  EXPECT_DOUBLE_EQ(0.0, position_of(1));
  EXPECT_DOUBLE_EQ(0.0, velocity_of(1));
}

TEST_F(DummyAllModesTest, CurrentBasedPositionTracksPositionAndReportsCap)
{
  ASSERT_TRUE(
    driver_.set_control_mode(1, dynamixel_hardware::ControlMode::CurrentBasedPosition));
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_positions({1}, {7.0}));  // beyond pi: CBP is multi-turn, no clamp
  ASSERT_TRUE(driver_.write_efforts({1}, {300.0}));
  EXPECT_DOUBLE_EQ(7.0, position_of(1));
  EXPECT_DOUBLE_EQ(300.0, effort_of(1));
}

TEST_F(DummyAllModesTest, PwmModeMirrorsDutyToEffort)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::PWM));
  driver_.tick(0.1);
  ASSERT_TRUE(driver_.write_pwms({1}, {0.25}));
  EXPECT_DOUBLE_EQ(0.25, effort_of(1));
  EXPECT_DOUBLE_EQ(0.0, position_of(1));
}

TEST_F(DummyAllModesTest, RejectsWritesNotMatchingTheActiveMode)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Velocity));
  EXPECT_FALSE(driver_.write_positions({1}, {1.0}));
  EXPECT_FALSE(driver_.last_error().empty());
  EXPECT_FALSE(driver_.write_pwms({1}, {0.5}));
  EXPECT_FALSE(driver_.write_efforts({1}, {10.0}));
}

TEST_F(DummyAllModesTest, ModeSwitchClearsVelocityGoal)
{
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Velocity));
  ASSERT_TRUE(driver_.write_velocities({1}, {1.0}));
  driver_.tick(0.1);
  const double moved = position_of(1);
  EXPECT_NEAR(0.1, moved, 1e-9);
  ASSERT_TRUE(driver_.set_control_mode(1, dynamixel_hardware::ControlMode::Position));
  driver_.tick(0.1);
  EXPECT_DOUBLE_EQ(moved, position_of(1));  // no further integration after the switch
}

}  // namespace
