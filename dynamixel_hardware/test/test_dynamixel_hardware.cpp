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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include "dynamixel_hardware/compat.hpp"
#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include "mock_driver.hpp"

#if DXL_HAS_PARAMS_ON_INIT
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#endif

namespace
{

using dynamixel_hardware::ControlMode;
using dynamixel_hardware::DynamixelHardware;
using dynamixel_hardware::MockDriver;
using ::testing::_;
using ::testing::DoAll;
using ::testing::DoubleEq;
using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::SetArgReferee;

constexpr char kValidSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="port_name">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <param name="Profile_Velocity">100</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint2">
      <param name="id">2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

constexpr char kUsbPortSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="usb_port">/dev/ttyUSB1</param>
      <param name="baud_rate">57600</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint2">
      <param name="id">2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

constexpr char kDummySystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="use_dummy">true</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint2">
      <param name="id">2</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
)";

constexpr char kMissingIdSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="port_name">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
)";

constexpr char kBadIdSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="port_name">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
    </hardware>
    <joint name="joint1">
      <param name="id">abc</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
)";

constexpr char kBadBaudRateSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="port_name">/dev/ttyUSB0</param>
      <param name="baud_rate">fast</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
)";

constexpr char kNoPortSystem[] =
  R"(
  <ros2_control name="DynamixelHardware" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="baud_rate">1000000</param>
    </hardware>
    <joint name="joint1">
      <param name="id">1</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
)";

std::string make_urdf(const std::string & ros2_control_snippet)
{
  return std::string(ros2_control_test_assets::urdf_head) + ros2_control_snippet +
         std::string(ros2_control_test_assets::urdf_tail);
}

hardware_interface::HardwareInfo parse_info(const std::string & ros2_control_snippet)
{
  const auto infos =
    hardware_interface::parse_control_resources_from_urdf(make_urdf(ros2_control_snippet));
  return infos[0];
}

CallbackReturn call_on_init(
  DynamixelHardware & hw, const hardware_interface::HardwareInfo & info)
{
#if DXL_HAS_PARAMS_ON_INIT
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  return hw.on_init(params);
#else
  return hw.on_init(info);
#endif
}

// Default ON_CALLs shared by the mock-driver fixtures: every driver call
// succeeds. read_states() is deliberately left out -- each fixture decides
// what its servos report.
void set_default_driver_actions(MockDriver & mock)
{
  ON_CALL(mock, connect(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, ping(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, setup(_)).WillByDefault(Return(true));
  ON_CALL(mock, set_torque(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, set_control_mode(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, write_item(_, _, _)).WillByDefault(Return(true));
  ON_CALL(mock, write_positions(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, write_velocities(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, write_efforts(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, write_pwms(_, _)).WillByDefault(Return(true));
  ON_CALL(mock, last_error()).WillByDefault(Return(std::string("mock error")));
}

class TestDynamixelHardware : public ::testing::Test
{
protected:
  void init_with_mock(const std::string & snippet)
  {
    info_ = parse_info(snippet);
    ASSERT_EQ(CallbackReturn::SUCCESS, call_on_init(hw_, info_));
    auto mock = std::make_unique<NiceMock<MockDriver>>();
    mock_ = mock.get();
    set_default_driver_actions(*mock_);
    ON_CALL(*mock_, read_states(_, _, _, _))
    .WillByDefault(
      DoAll(
        SetArgReferee<1>(std::vector<double>{0.5, 1.5}),
        SetArgReferee<2>(std::vector<double>{0.0, 0.0}),
        SetArgReferee<3>(std::vector<double>{0.0, 0.0}), Return(true)));
    hw_.set_driver_for_testing(std::move(mock));
  }

  void configure_and_activate()
  {
    ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
    ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_activate(rclcpp_lifecycle::State()));
    export_interfaces();
  }

  void export_interfaces()
  {
#if DXL_HAS_ON_EXPORT
    state_interfaces_ = hw_.on_export_state_interfaces();
    command_interfaces_ = hw_.on_export_command_interfaces();
#else
    state_interfaces_ = hw_.export_state_interfaces();
    command_interfaces_ = hw_.export_command_interfaces();
#endif
  }

  void set_command(const std::string & name, double value)
  {
#if DXL_HAS_ON_EXPORT
    for (auto & interface : command_interfaces_) {
      if (interface->get_name() == name) {
        // Loaned-handle value accessors are gated on DXL_HAS_PARAMS_ON_INIT
        // (bool-returning set_value), independent of the export-signature gate.
#if DXL_HAS_PARAMS_ON_INIT
        (void)interface->set_value(value);
#else
        interface->set_value(value);
#endif
        return;
      }
    }
#else
    for (auto & interface : command_interfaces_) {
      if (interface.get_name() == name) {
        interface.set_value(value);
        return;
      }
    }
#endif
    FAIL() << "unknown command interface: " << name;
  }

  double get_state(const std::string & name)
  {
#if DXL_HAS_ON_EXPORT
    for (const auto & interface : state_interfaces_) {
      if (interface->get_name() == name) {
        // get_optional() is gated on DXL_HAS_PARAMS_ON_INIT (get_optional is
        // 4.27.0; 4.34.0 discriminates the pinned targets), not the export gate.
#if DXL_HAS_PARAMS_ON_INIT
        return interface->get_optional().value();
#else
        return interface->get_value();
#endif
      }
    }
#else
    for (const auto & interface : state_interfaces_) {
      if (interface.get_name() == name) {
        return interface.get_value();
      }
    }
#endif
    ADD_FAILURE() << "unknown state interface: " << name;
    return 0.0;
  }

  return_type write_once()
  {
    return hw_.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));
  }

  return_type read_once()
  {
    return hw_.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.01));
  }

  DynamixelHardware hw_;
  hardware_interface::HardwareInfo info_;
  NiceMock<MockDriver> * mock_{nullptr};
#if DXL_HAS_ON_EXPORT
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
#else
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
#endif
};

// --- parameter parsing -----------------------------------------------------

TEST_F(TestDynamixelHardware, on_init_succeeds_with_valid_parameters)
{
  EXPECT_EQ(CallbackReturn::SUCCESS, call_on_init(hw_, parse_info(kValidSystem)));
}

TEST_F(TestDynamixelHardware, on_init_fails_without_joint_id)
{
  EXPECT_EQ(CallbackReturn::ERROR, call_on_init(hw_, parse_info(kMissingIdSystem)));
}

TEST_F(TestDynamixelHardware, on_init_fails_with_non_numeric_id)
{
  EXPECT_EQ(CallbackReturn::ERROR, call_on_init(hw_, parse_info(kBadIdSystem)));
}

TEST_F(TestDynamixelHardware, on_init_fails_with_non_numeric_baud_rate)
{
  EXPECT_EQ(CallbackReturn::ERROR, call_on_init(hw_, parse_info(kBadBaudRateSystem)));
}

TEST_F(TestDynamixelHardware, on_init_fails_without_port)
{
  EXPECT_EQ(CallbackReturn::ERROR, call_on_init(hw_, parse_info(kNoPortSystem)));
}

// --- lifecycle sequencing --------------------------------------------------

TEST_F(TestDynamixelHardware, on_configure_connects_pings_and_sets_up)
{
  init_with_mock(kValidSystem);
  InSequence seq;
  EXPECT_CALL(*mock_, connect("/dev/ttyUSB0", 1000000)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, ping(1, _)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, ping(2, _)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, setup(std::vector<uint8_t>{1, 2})).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_item(1, "Profile_Velocity", 100)).WillOnce(Return(true));
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_configure_fails_when_ping_fails)
{
  init_with_mock(kValidSystem);
  EXPECT_CALL(*mock_, connect(_, _)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, ping(1, _)).WillOnce(Return(false));
  EXPECT_CALL(*mock_, setup(_)).Times(0);
  EXPECT_EQ(CallbackReturn::ERROR, hw_.on_configure(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_configure_fails_when_setup_fails)
{
  init_with_mock(kValidSystem);
  EXPECT_CALL(*mock_, connect(_, _)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, ping(_, _)).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_, setup(_)).WillOnce(Return(false));
  EXPECT_CALL(*mock_, set_control_mode(_, _)).Times(0);
  EXPECT_EQ(CallbackReturn::ERROR, hw_.on_configure(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_configure_uses_usb_port_fallback)
{
  init_with_mock(kUsbPortSystem);
  EXPECT_CALL(*mock_, connect("/dev/ttyUSB1", 57600)).WillOnce(Return(true));
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_activate_reads_states_resets_commands_and_enables_torque)
{
  init_with_mock(kValidSystem);
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
  {
    InSequence seq;
    EXPECT_CALL(*mock_, read_states(std::vector<uint8_t>{1, 2}, _, _, _))
    .WillOnce(
      DoAll(
        SetArgReferee<1>(std::vector<double>{0.5, 1.5}),
        SetArgReferee<2>(std::vector<double>{0.0, 0.0}),
        SetArgReferee<3>(std::vector<double>{0.0, 0.0}), Return(true)));
    EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_torque(2, true)).WillOnce(Return(true));
  }
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_activate(rclcpp_lifecycle::State()));

  // reset_command() synchronized commands to the read states, so the very
  // next write() re-sends the current positions (no NaN, no zero jump).
  EXPECT_CALL(*mock_, tick(_));
  EXPECT_CALL(*mock_, write_positions(_, ElementsAre(DoubleEq(0.5), DoubleEq(1.5))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

// Regression: a failed initial sync-read must not leave the NaN state that
// init_impl() seeds every joint with flowing into reset_command() and then
// out to torque-enabled servos on the first write() (NaN != NaN, so the
// change-detection in write() would treat it as a real command). on_activate
// must refuse to enable torque when the post-read state is still NaN.
TEST_F(TestDynamixelHardware, on_activate_fails_when_initial_read_fails)
{
  init_with_mock(kValidSystem);
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
  EXPECT_CALL(*mock_, read_states(std::vector<uint8_t>{1, 2}, _, _, _)).WillOnce(Return(false));
  EXPECT_CALL(*mock_, set_torque(_, true)).Times(0);
  EXPECT_EQ(CallbackReturn::ERROR, hw_.on_activate(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_deactivate_disables_torque)
{
  init_with_mock(kValidSystem);
  configure_and_activate();
  EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_deactivate(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_cleanup_and_shutdown_disconnect)
{
  init_with_mock(kValidSystem);
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
  EXPECT_CALL(*mock_, disconnect()).Times(2);
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_cleanup(rclcpp_lifecycle::State()));
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_shutdown(rclcpp_lifecycle::State()));
}

TEST_F(TestDynamixelHardware, on_error_disables_torque_and_disconnects_best_effort)
{
  init_with_mock(kValidSystem);
  configure_and_activate();
  InSequence seq;
  EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(false));  // failure tolerated
  EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, disconnect());
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_error(rclcpp_lifecycle::State()));
}

// init_impl()'s `if (!driver_)` guard is a binding contract: an injected
// driver (set_driver_for_testing() called before on_init) must survive
// on_init, because M3/M4 test fixtures rely on init_impl only constructing a
// driver when none is already set. Every other test in this file goes
// through init_with_mock(), which calls on_init first and injects second --
// that only exercises the "driver_ was null" branch. This test drives the
// opposite order on purpose and asserts the *injected* mock (not a freshly
// constructed WorkbenchDriver) is what on_configure() drives. Do not
// "simplify" this to match init_with_mock()'s ordering: doing so would
// silently stop covering the guard, and a later unconditional
// `driver_ = std::make_unique<WorkbenchDriver>()` in init_impl would leave
// all other tests green while M3/M4 fixtures start opening /dev/ttyUSB0.
TEST_F(TestDynamixelHardware, injected_driver_survives_on_init)
{
  auto mock = std::make_unique<NiceMock<MockDriver>>();
  auto * injected = mock.get();
  ON_CALL(*injected, ping(_, _)).WillByDefault(Return(true));
  ON_CALL(*injected, setup(_)).WillByDefault(Return(true));
  ON_CALL(*injected, set_control_mode(_, _)).WillByDefault(Return(true));
  ON_CALL(*injected, write_item(_, _, _)).WillByDefault(Return(true));
  hw_.set_driver_for_testing(std::move(mock));

  ASSERT_EQ(CallbackReturn::SUCCESS, call_on_init(hw_, parse_info(kValidSystem)));

  EXPECT_CALL(*injected, connect("/dev/ttyUSB0", 1000000)).WillOnce(Return(true));
  EXPECT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
}

// --- legacy heuristic mode switching in write() ----------------------------

TEST_F(TestDynamixelHardware, write_switches_all_joints_to_velocity_mode_on_velocity_command)
{
  init_with_mock(kValidSystem);
  configure_and_activate();
  // M3: the legacy write() heuristic only governs joints whose controller
  // claims position and velocity together.
  const std::vector<std::string> claims = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"};
  ASSERT_EQ(return_type::OK, hw_.prepare_command_mode_switch(claims, {}));
  ASSERT_EQ(return_type::OK, hw_.perform_command_mode_switch(claims, {}));
  set_command("joint1/velocity", 1.0);
  InSequence seq;
  EXPECT_CALL(*mock_, tick(_));
  // Dynamixel requires torque off around an operating-mode rewrite.
  EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Velocity)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Velocity)).WillOnce(Return(true));
  // Extra joint parameters are rewritten after every mode change, while
  // torque is still off (M3 ordering).
  EXPECT_CALL(*mock_, write_item(1, "Profile_Velocity", 100)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, true)).WillOnce(Return(true));
  // Legacy quirk kept on purpose: the mode switch resets the commands, so the
  // first velocity write after a switch sends zeros; the controller re-writes
  // its command on the next cycle.
  EXPECT_CALL(*mock_, write_velocities(_, ElementsAre(DoubleEq(0.0), DoubleEq(0.0))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(TestDynamixelHardware, write_sends_velocity_commands_once_in_velocity_mode)
{
  init_with_mock(kValidSystem);
  configure_and_activate();
  const std::vector<std::string> claims = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"};
  ASSERT_EQ(return_type::OK, hw_.prepare_command_mode_switch(claims, {}));
  ASSERT_EQ(return_type::OK, hw_.perform_command_mode_switch(claims, {}));
  set_command("joint1/velocity", 1.0);
  ASSERT_EQ(return_type::OK, write_once());  // switches to velocity mode

  set_command("joint1/velocity", 1.0);
  set_command("joint2/velocity", -0.5);
  EXPECT_CALL(*mock_, set_torque(_, _)).Times(0);
  EXPECT_CALL(*mock_, set_control_mode(_, _)).Times(0);
  EXPECT_CALL(*mock_, write_velocities(_, ElementsAre(DoubleEq(1.0), DoubleEq(-0.5))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(TestDynamixelHardware, write_switches_back_to_position_mode_on_position_command)
{
  init_with_mock(kValidSystem);
  configure_and_activate();
  const std::vector<std::string> claims = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"};
  ASSERT_EQ(return_type::OK, hw_.prepare_command_mode_switch(claims, {}));
  ASSERT_EQ(return_type::OK, hw_.perform_command_mode_switch(claims, {}));
  set_command("joint1/velocity", 1.0);
  ASSERT_EQ(return_type::OK, write_once());  // now in velocity mode

  set_command("joint1/position", 0.7);
  InSequence seq;
  EXPECT_CALL(*mock_, tick(_));
  EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_item(1, "Profile_Velocity", 100)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, true)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_positions(_, _)).WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

// --- dummy end-to-end through the plugin ------------------------------------

TEST_F(TestDynamixelHardware, dummy_mode_reflects_position_commands)
{
  info_ = parse_info(kDummySystem);
  ASSERT_EQ(CallbackReturn::SUCCESS, call_on_init(hw_, info_));
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_activate(rclcpp_lifecycle::State()));
  export_interfaces();
  set_command("joint1/position", 0.7);
  ASSERT_EQ(return_type::OK, write_once());
  ASSERT_EQ(return_type::OK, read_once());
  EXPECT_DOUBLE_EQ(0.7, get_state("joint1/position"));
}

// End-to-end regression for
// https://github.com/dynamixel-community/dynamixel_hardware/issues/71.
TEST_F(TestDynamixelHardware, dummy_mode_integrates_velocity_commands)
{
  info_ = parse_info(kDummySystem);
  ASSERT_EQ(CallbackReturn::SUCCESS, call_on_init(hw_, info_));
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_configure(rclcpp_lifecycle::State()));
  ASSERT_EQ(CallbackReturn::SUCCESS, hw_.on_activate(rclcpp_lifecycle::State()));
  export_interfaces();
  const std::vector<std::string> claims = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"};
  ASSERT_EQ(return_type::OK, hw_.prepare_command_mode_switch(claims, {}));
  ASSERT_EQ(return_type::OK, hw_.perform_command_mode_switch(claims, {}));
  set_command("joint1/velocity", 0.5);
  ASSERT_EQ(return_type::OK, write_once());  // switches to velocity mode
  for (int i = 0; i < 100; i++) {
    set_command("joint1/velocity", 0.5);
    ASSERT_EQ(return_type::OK, write_once());
  }
  ASSERT_EQ(return_type::OK, read_once());
  EXPECT_NEAR(0.5, get_state("joint1/velocity"), 1e-9);
  EXPECT_GT(get_state("joint1/position"), 0.4);
}

// ---------------------------------------------------------------------------
// M3: per-joint mode selection/switching, capability rejection, effort unit
// conversion, the pwm interface, the legacy heuristic and the #69 mixed-mode
// regression. Two layers:
//  - ControlModeM3Test: MockDriver injected into a directly instantiated
//    plugin (driver-call assertions).
//  - DummyFullStackM3Test: plugin loaded from URDF through a ResourceManager
//    with use_dummy=true (full-stack behavior assertions).
// ---------------------------------------------------------------------------

namespace m3_test
{

// Both joints declare every command interface the plugin exports, so each
// test can claim exactly the ones its scenario needs.
std::string ros2_control_block(
  const std::string & hardware_params, const std::string & joint1_params,
  const std::string & joint2_params)
{
  const std::string hardware_head =
    R"(
  <ros2_control name="dxl" type="system">
    <hardware>
      <plugin>dynamixel_hardware/DynamixelHardware</plugin>
      <param name="port_name">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
)";
  const std::string hardware_tail = R"(    </hardware>
)";
  const std::string joint1_head = R"(    <joint name="joint1">
      <param name="id">1</param>
)";
  const std::string joint2_head = R"(    <joint name="joint2">
      <param name="id">2</param>
)";
  const std::string joint_tail =
    R"(      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <command_interface name="pwm"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
)";
  return hardware_head + hardware_params + hardware_tail + joint1_head + joint1_params +
         joint_tail + joint2_head + joint2_params + joint_tail + "  </ros2_control>\n";
}

class ControlModeM3Test : public ::testing::Test
{
protected:
  CallbackReturn try_init(const std::string & joint1_params, const std::string & joint2_params)
  {
    hw_ = std::make_unique<DynamixelHardware>();
    auto mock = std::make_unique<NiceMock<MockDriver>>();
    mock_ = mock.get();
    set_default_driver_actions(*mock_);
    // Every servo reports zeros; the joint count follows the requested ids.
    ON_CALL(*mock_, read_states(_, _, _, _))
    .WillByDefault(
      Invoke(
        [](const std::vector<uint8_t> & ids, std::vector<double> & positions,
        std::vector<double> & velocities, std::vector<double> & efforts) {
          positions.assign(ids.size(), 0.0);
          velocities.assign(ids.size(), 0.0);
          efforts.assign(ids.size(), 0.0);
          return true;
        }));
    hw_->set_driver_for_testing(std::move(mock));
    return call_on_init(*hw_, parse_info(ros2_control_block("", joint1_params, joint2_params)));
  }

  void init(const std::string & joint1_params, const std::string & joint2_params)
  {
    ASSERT_EQ(CallbackReturn::SUCCESS, try_init(joint1_params, joint2_params));
  }

  void configure_and_activate()
  {
    ASSERT_EQ(CallbackReturn::SUCCESS, hw_->on_configure(rclcpp_lifecycle::State()));
    ASSERT_EQ(CallbackReturn::SUCCESS, hw_->on_activate(rclcpp_lifecycle::State()));
    refresh_handles();
  }

  void refresh_handles()
  {
#if DXL_HAS_ON_EXPORT
    command_handles_ = hw_->on_export_command_interfaces();
    state_handles_ = hw_->on_export_state_interfaces();
#else
    command_handles_ = hw_->export_command_interfaces();
    state_handles_ = hw_->export_state_interfaces();
#endif
  }

  // Export order is fixed by DynamixelHardware: commands per joint are
  // position(0), velocity(1), effort(2), pwm(3); states per joint are
  // position(0), velocity(1), effort(2).
  void set_command(size_t joint, size_t offset, double value)
  {
#if DXL_HAS_PARAMS_ON_INIT
    EXPECT_TRUE(command_handles_[joint * 4 + offset]->set_value(value));
#elif DXL_HAS_ON_EXPORT
    command_handles_[joint * 4 + offset]->set_value(value);
#else
    command_handles_[joint * 4 + offset].set_value(value);
#endif
  }

  double state_value(size_t joint, size_t offset)
  {
#if DXL_HAS_PARAMS_ON_INIT
    return state_handles_[joint * 3 + offset]->get_optional().value();
#elif DXL_HAS_ON_EXPORT
    return state_handles_[joint * 3 + offset]->get_value();
#else
    return state_handles_[joint * 3 + offset].get_value();
#endif
  }

  return_type prepare_perform(
    const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    const auto prepared = hw_->prepare_command_mode_switch(start, stop);
    if (prepared != return_type::OK) {
      return prepared;
    }
    return hw_->perform_command_mode_switch(start, stop);
  }

  return_type write_once()
  {
    return hw_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  }

  std::unique_ptr<DynamixelHardware> hw_;
  NiceMock<MockDriver> * mock_{nullptr};
#if DXL_HAS_ON_EXPORT
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_handles_;
#else
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
#endif
};

TEST_F(ControlModeM3Test, configure_applies_every_configured_control_mode)
{
  const std::vector<std::pair<std::string, ControlMode>> cases = {
    {"position", ControlMode::Position},
    {"extended_position", ControlMode::ExtendedPosition},
    {"multi_turn", ControlMode::MultiTurn},
    {"current_based_position", ControlMode::CurrentBasedPosition},
    {"velocity", ControlMode::Velocity},
    {"current", ControlMode::Current},
    {"torque", ControlMode::Torque},
    {"pwm", ControlMode::PWM},
  };
  for (const auto & mode_case : cases) {
    SCOPED_TRACE(mode_case.first);
    init("      <param name=\"control_mode\">" + mode_case.first + "</param>\n", "");
    EXPECT_CALL(*mock_, set_control_mode(1, mode_case.second)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Position)).WillOnce(Return(true));
    ASSERT_EQ(CallbackReturn::SUCCESS, hw_->on_configure(rclcpp_lifecycle::State()));
  }
}

TEST_F(ControlModeM3Test, unknown_control_mode_value_fails_on_init)
{
  EXPECT_EQ(
    CallbackReturn::ERROR, try_init("      <param name=\"control_mode\">banana</param>\n", ""));
}

TEST_F(ControlModeM3Test, malformed_torque_constant_fails_on_init)
{
  EXPECT_EQ(
    CallbackReturn::ERROR, try_init("      <param name=\"torque_constant\">abc</param>\n", ""));
  EXPECT_EQ(
    CallbackReturn::ERROR, try_init("      <param name=\"torque_constant\">-1.5</param>\n", ""));
}

TEST_F(ControlModeM3Test, velocity_claim_switches_mode_with_torque_sequencing)
{
  init("", "");
  configure_and_activate();
  EXPECT_CALL(*mock_, set_control_mode(2, _)).Times(0);
  {
    InSequence sequence;
    EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Velocity)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
  }
  EXPECT_EQ(return_type::OK, prepare_perform({"joint1/velocity"}, {}));
}

TEST_F(ControlModeM3Test, effort_claim_uses_current_unless_configured_torque)
{
  init("", "      <param name=\"control_mode\">torque</param>\n");
  configure_and_activate();
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Current)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(2, _)).Times(0);  // already Torque; claim keeps it
  EXPECT_EQ(return_type::OK, prepare_perform({"joint1/effort", "joint2/effort"}, {}));
}

TEST_F(ControlModeM3Test, position_plus_effort_claim_selects_current_based_position)
{
  init("", "");
  configure_and_activate();
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::CurrentBasedPosition))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, prepare_perform({"joint1/position", "joint1/effort"}, {}));
}

TEST_F(ControlModeM3Test, position_claim_keeps_configured_position_family_variant)
{
  init("      <param name=\"control_mode\">extended_position</param>\n", "");
  configure_and_activate();  // joint1 already switched to ExtendedPosition here
  EXPECT_CALL(*mock_, set_control_mode(1, _)).Times(0);
  EXPECT_EQ(return_type::OK, prepare_perform({"joint1/position"}, {}));
}

TEST_F(ControlModeM3Test, position_claim_falls_back_to_position_for_non_position_family_config)
{
  init("      <param name=\"control_mode\">velocity</param>\n", "");
  configure_and_activate();  // joint1 active mode is Velocity
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, prepare_perform({"joint1/position"}, {}));
}

TEST_F(ControlModeM3Test, invalid_interface_combination_is_rejected)
{
  init("", "");
  configure_and_activate();
  EXPECT_EQ(
    return_type::ERROR,
    hw_->prepare_command_mode_switch({"joint1/velocity", "joint1/effort"}, {}));
  EXPECT_EQ(
    return_type::ERROR, hw_->prepare_command_mode_switch({"joint1/pwm", "joint1/position"}, {}));
}

TEST_F(ControlModeM3Test, configure_fails_when_driver_rejects_configured_mode)
{
  init("      <param name=\"control_mode\">current</param>\n", "");
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Current)).WillOnce(Return(false));
  EXPECT_EQ(CallbackReturn::ERROR, hw_->on_configure(rclcpp_lifecycle::State()));
}

TEST_F(ControlModeM3Test, perform_fails_when_driver_rejects_requested_mode)
{
  init("", "");
  configure_and_activate();
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Current)).WillOnce(Return(false));
  EXPECT_EQ(return_type::OK, hw_->prepare_command_mode_switch({"joint1/effort"}, {}));
  EXPECT_EQ(return_type::ERROR, hw_->perform_command_mode_switch({"joint1/effort"}, {}));
}

TEST_F(ControlModeM3Test, torque_constant_converts_effort_command_to_milliamps)
{
  init(
    "      <param name=\"control_mode\">current</param>\n"
    "      <param name=\"torque_constant\">2.0</param>\n", "");
  configure_and_activate();
  ASSERT_EQ(return_type::OK, prepare_perform({"joint1/effort"}, {}));
  set_command(0, 2, 0.5);  // 0.5 Nm -> 0.5 * 1000 / 2.0 = 250 mA
  EXPECT_CALL(*mock_, write_efforts(ElementsAre(1), ElementsAre(DoubleNear(250.0, 1e-9))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(ControlModeM3Test, torque_constant_converts_effort_state_to_newton_metres)
{
  init(
    "      <param name=\"control_mode\">current</param>\n"
    "      <param name=\"torque_constant\">2.0</param>\n", "");
  configure_and_activate();
  ON_CALL(*mock_, read_states(_, _, _, _))
  .WillByDefault(
    Invoke(
      [](const std::vector<uint8_t> & ids, std::vector<double> & positions,
      std::vector<double> & velocities, std::vector<double> & efforts) {
        positions.assign(ids.size(), 0.0);
        velocities.assign(ids.size(), 0.0);
        efforts.assign(ids.size(), 250.0);  // motor-side mA
        return true;
      }));
  ASSERT_EQ(return_type::OK, hw_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)));
  EXPECT_NEAR(0.5, state_value(0, 2), 1e-9);  // 250 mA -> 0.5 Nm
}

TEST_F(ControlModeM3Test, mixed_position_and_velocity_joints_are_batched_separately)
{
  // Regression for #69: one joint in position mode, another in velocity mode,
  // simultaneously active in a single write() cycle.
  init("", "");
  configure_and_activate();
  ASSERT_EQ(return_type::OK, prepare_perform({"joint1/position", "joint2/velocity"}, {}));
  set_command(0, 0, 0.7);
  set_command(1, 1, 0.4);
  EXPECT_CALL(*mock_, write_positions(ElementsAre(1), ElementsAre(DoubleNear(0.7, 1e-12))))
  .WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_velocities(ElementsAre(2), ElementsAre(DoubleNear(0.4, 1e-12))))
  .WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_efforts(_, _)).Times(0);
  EXPECT_CALL(*mock_, write_pwms(_, _)).Times(0);
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(ControlModeM3Test, current_based_position_writes_position_and_cap_batches)
{
  init("", "");
  configure_and_activate();
  ASSERT_EQ(return_type::OK, prepare_perform({"joint1/position", "joint1/effort"}, {}));
  set_command(0, 0, 1.2);
  set_command(0, 2, 300.0);  // no torque_constant -> plain mA cap
  // joint2 stays in Position mode and is batched together with joint1.
  EXPECT_CALL(
    *mock_,
    write_positions(
      ElementsAre(1, 2), ElementsAre(DoubleNear(1.2, 1e-12), DoubleNear(0.0, 1e-12))))
  .WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_efforts(ElementsAre(1), ElementsAre(DoubleNear(300.0, 1e-12))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(ControlModeM3Test, pwm_claim_dispatches_to_write_pwms)
{
  init("      <param name=\"control_mode\">pwm</param>\n", "");
  configure_and_activate();
  ASSERT_EQ(return_type::OK, prepare_perform({"joint1/pwm"}, {}));
  set_command(0, 3, 0.25);
  EXPECT_CALL(*mock_, write_pwms(ElementsAre(1), ElementsAre(DoubleNear(0.25, 1e-12))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(ControlModeM3Test, position_plus_velocity_claim_keeps_legacy_heuristic)
{
  init("", "");
  configure_and_activate();
  EXPECT_CALL(*mock_, set_control_mode(_, _)).Times(0);
  ASSERT_EQ(
    return_type::OK,
    prepare_perform(
      {"joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"}, {}));
  ::testing::Mock::VerifyAndClearExpectations(mock_);
  // A velocity-command change now switches both legacy joints to Velocity
  // (historical write() heuristic), with torque sequencing.
  set_command(0, 1, 0.3);
  set_command(1, 1, 0.3);
  {
    InSequence sequence;
    EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Velocity)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Velocity)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
    EXPECT_CALL(*mock_, set_torque(2, true)).WillOnce(Return(true));
  }
  EXPECT_CALL(*mock_, write_velocities(ElementsAre(1, 2), _)).WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

class DummyFullStackM3Test : public ::testing::Test
{
protected:
  void start(const std::string & joint1_params, const std::string & joint2_params)
  {
    const std::string urdf = make_urdf(
      ros2_control_block(
        "      <param name=\"use_dummy\">true</param>\n", joint1_params, joint2_params));
// The ResourceManagerParams-struct ctor is 4.34.0+ — gate on
// DXL_HAS_PARAMS_ON_INIT (DXL_HAS_RM_PARAMS_CTOR covers only the
// (urdf, clock_iface, logger_iface) ctor, which this fixture does not use).
#if DXL_HAS_PARAMS_ON_INIT
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf;
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("test_dynamixel_hardware_m3_rm");
    rm_ = std::make_unique<hardware_interface::ResourceManager>(params, true);
#else
    rm_ = std::make_unique<hardware_interface::ResourceManager>(urdf, true, false);
#endif
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    ASSERT_EQ(return_type::OK, rm_->set_component_state("dxl", active));
  }

  // Loaned-handle get_optional()/bool set_value() are gated on
  // DXL_HAS_PARAMS_ON_INIT (get_optional is 4.27.0; 4.34.0 discriminates our
  // pinned humble 2.54.0 from jazzy/rolling), NOT on DXL_HAS_COMPONENT_LOGGER.
  static void set_cmd(hardware_interface::LoanedCommandInterface & iface, double value)
  {
#if DXL_HAS_PARAMS_ON_INIT
    EXPECT_TRUE(iface.set_value(value));
#else
    iface.set_value(value);
#endif
  }

  static double get_state(const hardware_interface::LoanedStateInterface & iface)
  {
#if DXL_HAS_PARAMS_ON_INIT
    return iface.get_optional().value();
#else
    return iface.get_value();
#endif
  }

  void cycle(double period_sec = 0.1)
  {
    (void)rm_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(period_sec));
    (void)rm_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(period_sec));
  }

  std::unique_ptr<hardware_interface::ResourceManager> rm_;
};

TEST_F(DummyFullStackM3Test, pwm_interface_exported_and_mirrored_to_effort)
{
  start("      <param name=\"control_mode\">pwm</param>\n", "");
  ASSERT_TRUE(rm_->prepare_command_mode_switch({"joint1/pwm"}, {}));
  ASSERT_TRUE(rm_->perform_command_mode_switch({"joint1/pwm"}, {}));
  auto pwm_cmd = rm_->claim_command_interface("joint1/pwm");
  auto effort_state = rm_->claim_state_interface("joint1/effort");
  set_cmd(pwm_cmd, 0.25);
  cycle();
  EXPECT_DOUBLE_EQ(0.25, get_state(effort_state));
}

TEST_F(DummyFullStackM3Test, mixed_position_and_velocity_joints_run_simultaneously)
{
  // Full-stack #69 regression: joint1 position-controlled while joint2 is
  // velocity-controlled in the same update cycles.
  start("", "");
  ASSERT_TRUE(rm_->prepare_command_mode_switch({"joint1/position", "joint2/velocity"}, {}));
  ASSERT_TRUE(rm_->perform_command_mode_switch({"joint1/position", "joint2/velocity"}, {}));
  auto j1_pos_cmd = rm_->claim_command_interface("joint1/position");
  auto j2_vel_cmd = rm_->claim_command_interface("joint2/velocity");
  auto j1_pos = rm_->claim_state_interface("joint1/position");
  auto j2_pos = rm_->claim_state_interface("joint2/position");
  auto j2_vel = rm_->claim_state_interface("joint2/velocity");
  for (int i = 0; i < 5; ++i) {
    set_cmd(j1_pos_cmd, 1.0);
    set_cmd(j2_vel_cmd, 0.5);
    cycle();
  }
  EXPECT_DOUBLE_EQ(1.0, get_state(j1_pos));
  // Goal arrives in cycle 1; cycles 2-5 integrate it: 4 * 0.1 * 0.5 = 0.2.
  EXPECT_NEAR(0.2, get_state(j2_pos), 1e-9);
  EXPECT_DOUBLE_EQ(0.5, get_state(j2_vel));
}

TEST_F(DummyFullStackM3Test, legacy_heuristic_drives_velocity_then_position_phases)
{
  start("", "");
  const std::vector<std::string> claims = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"};
  ASSERT_TRUE(rm_->prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(rm_->perform_command_mode_switch(claims, {}));
  auto j1_pos_cmd = rm_->claim_command_interface("joint1/position");
  auto j2_pos_cmd = rm_->claim_command_interface("joint2/position");
  auto j1_vel_cmd = rm_->claim_command_interface("joint1/velocity");
  auto j2_vel_cmd = rm_->claim_command_interface("joint2/velocity");
  auto j1_pos = rm_->claim_state_interface("joint1/position");
  auto j2_pos = rm_->claim_state_interface("joint2/position");
  // Velocity phase: cycle 1 switches the mode (command reset clears the
  // goal), cycle 2 delivers the goal, cycles 3-5 integrate: 3 * 0.1 = 0.3.
  for (int i = 0; i < 5; ++i) {
    set_cmd(j1_vel_cmd, 1.0);
    set_cmd(j2_vel_cmd, 1.0);
    cycle();
  }
  EXPECT_NEAR(0.3, get_state(j1_pos), 1e-9);
  EXPECT_NEAR(0.3, get_state(j2_pos), 1e-9);
  // Position phase: a position-command change switches back; the controller
  // re-writes its command every cycle (mode-switch reset overwrites it once).
  for (int i = 0; i < 3; ++i) {
    set_cmd(j1_pos_cmd, 0.5);
    set_cmd(j2_pos_cmd, -0.3);
    cycle();
  }
  EXPECT_DOUBLE_EQ(0.5, get_state(j1_pos));
  EXPECT_DOUBLE_EQ(-0.3, get_state(j2_pos));
}

TEST_F(DummyFullStackM3Test, effort_round_trips_through_torque_constant)
{
  start(
    "      <param name=\"control_mode\">current</param>\n"
    "      <param name=\"torque_constant\">2.0</param>\n", "");
  ASSERT_TRUE(rm_->prepare_command_mode_switch({"joint1/effort"}, {}));
  ASSERT_TRUE(rm_->perform_command_mode_switch({"joint1/effort"}, {}));
  auto effort_cmd = rm_->claim_command_interface("joint1/effort");
  auto effort_state = rm_->claim_state_interface("joint1/effort");
  set_cmd(effort_cmd, 0.5);  // Nm; the dummy stores 250 mA; read converts back
  cycle();
  EXPECT_NEAR(0.5, get_state(effort_state), 1e-9);
}

}  // namespace m3_test

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
