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
#include <pty.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "dynamixel_hardware/workbench_driver.hpp"

namespace
{

using dynamixel_hardware::WorkbenchDriver;

// Opens a pseudo-terminal pair and returns the slave side's device path.
// DynamixelWorkbench::init() only opens and configures the serial port
// (termios), it never talks to a real device, so a pty slave lets these
// tests get WorkbenchDriver::connect() to genuinely succeed without any
// Dynamixel hardware attached. The master fd is intentionally leaked for the
// life of the test process: closing it would make the slave path invalid,
// and these tests never send or receive bytes on it.
std::string open_fake_serial_port()
{
  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[256] = {};
  if (openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) != 0) {
    return "";
  }
  return std::string(slave_name);
}

ControlItem make_item(uint16_t address, uint8_t data_length)
{
  ControlItem item;
  item.address = address;
  item.data_length = data_length;
  return item;
}

TEST(TestWorkbenchDriver, read_window_starts_at_min_of_position_and_current_addresses)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  // Current/load block below the position block (Protocol 2.0 layout).
  WorkbenchDriver::compute_read_window(
    make_item(132, 4), make_item(128, 4), make_item(126, 2), start_address, read_length);
  EXPECT_EQ(126u, start_address);
  // Position block below the current/load block (Protocol 1.0 layout).
  WorkbenchDriver::compute_read_window(
    make_item(36, 2), make_item(38, 2), make_item(40, 2), start_address, read_length);
  EXPECT_EQ(36u, start_address);
}

TEST(TestWorkbenchDriver, read_window_length_includes_the_historical_two_byte_gap)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  // MX (Protocol 1.0): Present_Position 36 (2 bytes), Present_Speed 38
  // (2 bytes), Present_Load 40 (2 bytes) -> 2 + 2 + 2 + 2 gap = 8.
  WorkbenchDriver::compute_read_window(
    make_item(36, 2), make_item(38, 2), make_item(40, 2), start_address, read_length);
  EXPECT_EQ(8u, read_length);
}

// Representative X-series present block (e.g. XM430): Present_Current 126
// (2 bytes), Present_Velocity 128 (4 bytes), Present_Position 132 (4 bytes)
// -> one window starting at 126 of length 4 + 4 + 2 + 2 = 12.
TEST(TestWorkbenchDriver, read_window_covers_the_x_series_present_block)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  WorkbenchDriver::compute_read_window(
    make_item(132, 4), make_item(128, 4), make_item(126, 2), start_address, read_length);
  EXPECT_EQ(126u, start_address);
  EXPECT_EQ(12u, read_length);
}

TEST(TestWorkbenchDriver, read_window_degenerates_when_all_items_share_one_address)
{
  uint16_t start_address = 0;
  uint16_t read_length = 0;
  WorkbenchDriver::compute_read_window(
    make_item(100, 2), make_item(100, 2), make_item(100, 2), start_address, read_length);
  EXPECT_EQ(100u, start_address);
  EXPECT_EQ(8u, read_length);
}

// Regression: DynamixelWorkbench leaves its port/packet handler pointers
// uninitialized until init() runs, and its destructor dereferences them, so a
// WorkbenchDriver that is constructed and never connected must not own one.
// The plugin builds a driver in on_init, long before on_configure connects.
TEST(TestWorkbenchDriver, destroying_an_unconnected_driver_does_not_crash)
{
  WorkbenchDriver driver;
  EXPECT_EQ("", driver.last_error());
}

TEST(TestWorkbenchDriver, calls_before_connect_fail_with_not_connected)
{
  WorkbenchDriver driver;
  EXPECT_FALSE(driver.ping(1));
  EXPECT_EQ("not connected", driver.last_error());
  EXPECT_FALSE(driver.setup({1, 2}));
  EXPECT_EQ("not connected", driver.last_error());
  EXPECT_FALSE(driver.set_torque(1, true));
  EXPECT_FALSE(driver.set_control_mode(1, dynamixel_hardware::ControlMode::Position));
  EXPECT_FALSE(driver.write_positions({1}, {0.0}));
  EXPECT_FALSE(driver.write_velocities({1}, {0.0}));
  EXPECT_FALSE(driver.write_efforts({1}, {0.0}));
  EXPECT_FALSE(driver.write_pwms({1}, {0.0}));
  EXPECT_FALSE(driver.write_item(1, "Profile_Velocity", 100));
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  EXPECT_FALSE(driver.read_states({1}, positions, velocities, efforts));
  EXPECT_EQ("not connected", driver.last_error());
}

TEST(TestWorkbenchDriver, disconnect_without_connect_is_safe)
{
  WorkbenchDriver driver;
  driver.disconnect();
  driver.disconnect();
  EXPECT_EQ("", driver.last_error());
}

// Regression: getItemInfo() returns pointers owned by the DynamixelWorkbench,
// so read_states()/write_positions()/write_velocities() must refuse to run
// (rather than dereference stale or never-populated pointers, or sync-write
// through handler indices that setup() never registered) until setup() has
// actually completed. connect() alone -- even a genuinely successful one --
// must not be enough.
TEST(TestWorkbenchDriver, calls_after_connect_before_setup_fail_with_not_set_up)
{
  const std::string port = open_fake_serial_port();
  ASSERT_FALSE(port.empty());
  WorkbenchDriver driver;
  ASSERT_TRUE(driver.connect(port, 57600));

  EXPECT_FALSE(driver.write_positions({1}, {0.0}));
  EXPECT_EQ("not set up", driver.last_error());
  EXPECT_FALSE(driver.write_velocities({1}, {0.0}));
  EXPECT_EQ("not set up", driver.last_error());
  EXPECT_FALSE(driver.write_efforts({1}, {0.0}));
  EXPECT_EQ("not set up", driver.last_error());
  EXPECT_FALSE(driver.write_pwms({1}, {0.0}));
  EXPECT_EQ("not set up", driver.last_error());
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  EXPECT_FALSE(driver.read_states({1}, positions, velocities, efforts));
  EXPECT_EQ("not set up", driver.last_error());
}

// Regression: disconnect() must clear control_items_ (not just reset the
// workbench), so that even a driver that had -- hypothetically -- completed
// setup() before is left refusing read/write calls again afterward, rather
// than dereferencing pointers into the now-destroyed DynamixelWorkbench.
// setup() itself cannot succeed without a real servo attached (getItemInfo()
// needs a model pinged onto the workbench first), so this drives setup()
// through its failure path -- which must not leave stale/partial entries in
// control_items_ either -- then asserts the guarded calls are refused both
// before and after disconnect(), with ensure_workbench() taking priority
// once disconnected.
TEST(TestWorkbenchDriver, disconnect_after_setup_attempt_leaves_calls_refused)
{
  const std::string port = open_fake_serial_port();
  ASSERT_FALSE(port.empty());
  WorkbenchDriver driver;
  ASSERT_TRUE(driver.connect(port, 57600));

  EXPECT_FALSE(driver.setup({1}));
  EXPECT_FALSE(driver.write_positions({1}, {0.0}));
  EXPECT_EQ("not set up", driver.last_error());

  driver.disconnect();
  EXPECT_FALSE(driver.write_positions({1}, {0.0}));
  EXPECT_EQ("not connected", driver.last_error());
  EXPECT_FALSE(driver.write_velocities({1}, {0.0}));
  EXPECT_EQ("not connected", driver.last_error());
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  EXPECT_FALSE(driver.read_states({1}, positions, velocities, efforts));
  EXPECT_EQ("not connected", driver.last_error());
}

// ---------------------------------------------------------------------------
// Parked defect (Task 12): non-finite commands must be refused before any
// tick conversion, on all four write_* paths, regardless of connection
// state -- a non-finite command is always a caller bug worth reporting
// precisely rather than one masked by the connection-guard diagnostics.
// ---------------------------------------------------------------------------

TEST(TestWorkbenchDriver, non_finite_commands_are_rejected_before_any_conversion)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  WorkbenchDriver driver;

  EXPECT_FALSE(driver.write_positions({1}, {nan}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
  EXPECT_FALSE(driver.write_velocities({1}, {inf}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
  EXPECT_FALSE(driver.write_efforts({1}, {nan}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
  EXPECT_FALSE(driver.write_pwms({1}, {-inf}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));

  // A finite command still reaches the pre-existing connection guard, so the
  // new check cannot mask the "not connected" / "not set up" diagnostics.
  EXPECT_FALSE(driver.write_positions({1}, {0.0}));
  EXPECT_EQ("not connected", driver.last_error());
}

TEST(TestWorkbenchDriver, one_non_finite_element_rejects_the_whole_batch)
{
  WorkbenchDriver driver;
  EXPECT_FALSE(
    driver.write_positions({1, 2}, {0.0, std::numeric_limits<double>::quiet_NaN()}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
}

// Regression: the guard has to be applied in the type the conversion actually
// uses. Every write_* path narrows the double to float
// (static_cast<float>(values[i])) before handing it to
// convertRadian2Value()/convertVelocity2Value()/convertCurrent2Value(), which
// assign the result to an int32_t -- and converting a non-finite float to an
// integer type is undefined behavior, in a 100 Hz control loop. Any finite
// double above ~3.4e38 becomes inf as a float, so checking std::isfinite() on
// the double alone let the whole class through. Reachable from a diverging
// controller passing through ~1e39 for one cycle on its way to NaN, or from a
// pathological but accepted parameter (gear_ratio 1e300 is finite and
// non-zero; torque_constant 1e-300 is finite and positive).
TEST(TestWorkbenchDriver, finite_commands_that_overflow_float_are_rejected)
{
  WorkbenchDriver driver;

  EXPECT_FALSE(driver.write_positions({1}, {1e300}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("ID 1"));
  EXPECT_FALSE(driver.write_velocities({2}, {-1e300}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("ID 2"));
  EXPECT_FALSE(driver.write_efforts({3}, {1e300}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("ID 3"));
  // write_pwms() was never at risk -- duty_to_pwm_ticks() clamps to [-1, 1]
  // before converting -- but it shares the guard, so a duty ratio this far out
  // of range is now refused as the caller bug it is rather than silently
  // clamped to full duty.
  EXPECT_FALSE(driver.write_pwms({4}, {1e300}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("finite"));

  // A magnitude that survives the narrowing still reaches the pre-existing
  // connection guard, so the bound cannot swallow ordinary commands.
  EXPECT_FALSE(driver.write_positions({1}, {1e30}));
  EXPECT_EQ("not connected", driver.last_error());
}

// Regression: a mismatched ids/values length would otherwise index past the
// shorter vector inside the write_* loops (e.g. commands[i] against
// values[i]); the same guard that checks finiteness must catch this first.
TEST(TestWorkbenchDriver, mismatched_ids_and_values_length_is_rejected)
{
  WorkbenchDriver driver;
  EXPECT_FALSE(driver.write_positions({1, 2}, {0.0}));
  EXPECT_THAT(driver.last_error(), ::testing::HasSubstr("count"));
}

// ---------------------------------------------------------------------------
// M3 (feat/control-modes): pure-function tests for the new mode helpers.
// ---------------------------------------------------------------------------

TEST(WorkbenchDriverPureFunctionsM3, DutyToPwmTicksScalesAndClamps)
{
  EXPECT_EQ(885, dynamixel_hardware::WorkbenchDriver::duty_to_pwm_ticks(1.0));
  EXPECT_EQ(-885, dynamixel_hardware::WorkbenchDriver::duty_to_pwm_ticks(-1.0));
  EXPECT_EQ(0, dynamixel_hardware::WorkbenchDriver::duty_to_pwm_ticks(0.0));
  EXPECT_EQ(443, dynamixel_hardware::WorkbenchDriver::duty_to_pwm_ticks(0.5));
  EXPECT_EQ(885, dynamixel_hardware::WorkbenchDriver::duty_to_pwm_ticks(2.0));    // clamped
  EXPECT_EQ(-885, dynamixel_hardware::WorkbenchDriver::duty_to_pwm_ticks(-2.0));  // clamped
}

TEST(WorkbenchDriverPureFunctionsM3, RequiredItemPerMode)
{
  using dynamixel_hardware::ControlMode;
  using dynamixel_hardware::WorkbenchDriver;
  EXPECT_STREQ("Goal_Current", WorkbenchDriver::required_item_for(ControlMode::Current));
  EXPECT_STREQ(
    "Goal_Current", WorkbenchDriver::required_item_for(ControlMode::CurrentBasedPosition));
  EXPECT_STREQ("Goal_Torque", WorkbenchDriver::required_item_for(ControlMode::Torque));
  EXPECT_STREQ("Goal_PWM", WorkbenchDriver::required_item_for(ControlMode::PWM));
  EXPECT_EQ(nullptr, WorkbenchDriver::required_item_for(ControlMode::Position));
  EXPECT_EQ(nullptr, WorkbenchDriver::required_item_for(ControlMode::Velocity));
  EXPECT_EQ(nullptr, WorkbenchDriver::required_item_for(ControlMode::ExtendedPosition));
  EXPECT_EQ(nullptr, WorkbenchDriver::required_item_for(ControlMode::MultiTurn));
}

}  // namespace
