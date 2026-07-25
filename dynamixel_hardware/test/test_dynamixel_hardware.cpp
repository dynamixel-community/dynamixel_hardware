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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include "dynamixel_hardware/compat.hpp"
#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include "mock_driver.hpp"

#if DXL_HAS_PARAMS_ON_INIT
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#endif

namespace
{

using dynamixel_hardware::ControlMode;
using dynamixel_hardware::DynamixelHardware;
using dynamixel_hardware::MockDriver;
using ::testing::_;
using ::testing::DoAll;
using ::testing::DoubleEq;
using ::testing::ElementsAre;
using ::testing::InSequence;
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

hardware_interface::HardwareInfo parse_info(const std::string & ros2_control_snippet)
{
  const std::string urdf = std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_snippet + std::string(ros2_control_test_assets::urdf_tail);
  const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
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

class TestDynamixelHardware : public ::testing::Test
{
protected:
  void init_with_mock(const std::string & snippet)
  {
    info_ = parse_info(snippet);
    ASSERT_EQ(CallbackReturn::SUCCESS, call_on_init(hw_, info_));
    auto mock = std::make_unique<NiceMock<MockDriver>>();
    mock_ = mock.get();
    ON_CALL(*mock_, connect(_, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, ping(_, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, setup(_)).WillByDefault(Return(true));
    ON_CALL(*mock_, set_torque(_, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, set_control_mode(_, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, write_item(_, _, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, write_positions(_, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, write_velocities(_, _)).WillByDefault(Return(true));
    ON_CALL(*mock_, read_states(_, _, _, _))
    .WillByDefault(
      DoAll(
        SetArgReferee<1>(std::vector<double>{0.5, 1.5}),
        SetArgReferee<2>(std::vector<double>{0.0, 0.0}),
        SetArgReferee<3>(std::vector<double>{0.0, 0.0}), Return(true)));
    ON_CALL(*mock_, last_error()).WillByDefault(Return(std::string("mock error")));
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
  set_command("joint1/velocity", 1.0);
  InSequence seq;
  EXPECT_CALL(*mock_, tick(_));
  // Dynamixel requires torque off around an operating-mode rewrite.
  EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Velocity)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Velocity)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, true)).WillOnce(Return(true));
  // Extra joint parameters are rewritten after every mode change.
  EXPECT_CALL(*mock_, write_item(1, "Profile_Velocity", 100)).WillOnce(Return(true));
  // Legacy quirk kept on purpose: enable_torque(true) runs reset_command(),
  // so the first velocity write after a mode switch sends zeros; the
  // controller re-writes its command on the next cycle.
  EXPECT_CALL(*mock_, write_velocities(_, ElementsAre(DoubleEq(0.0), DoubleEq(0.0))))
  .WillOnce(Return(true));
  EXPECT_EQ(return_type::OK, write_once());
}

TEST_F(TestDynamixelHardware, write_sends_velocity_commands_once_in_velocity_mode)
{
  init_with_mock(kValidSystem);
  configure_and_activate();
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
  set_command("joint1/velocity", 1.0);
  ASSERT_EQ(return_type::OK, write_once());  // now in velocity mode

  set_command("joint1/position", 0.7);
  InSequence seq;
  EXPECT_CALL(*mock_, tick(_));
  EXPECT_CALL(*mock_, set_torque(1, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, false)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(1, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_control_mode(2, ControlMode::Position)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(1, true)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, set_torque(2, true)).WillOnce(Return(true));
  EXPECT_CALL(*mock_, write_item(1, "Profile_Velocity", 100)).WillOnce(Return(true));
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

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
