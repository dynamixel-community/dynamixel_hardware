# Copyright 2026 Yutaka Kondo <yutaka.kondo@youtalk.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Integration test bringing up DynamixelHardware in dummy mode via ros2_control_node."""

import os
import time
import unittest

from builtin_interfaces.msg import Duration

from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)

import launch

from launch.substitutions import Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import launch_testing.actions

import pytest

import rclpy

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TEST_DIR = os.path.dirname(__file__)
CONTROLLERS_YAML = os.path.join(TEST_DIR, 'test_controllers.yaml')


@pytest.mark.launch_test
def generate_test_description():
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            os.path.join(TEST_DIR, 'test_robot.urdf.xacro'),
            ' ',
            'use_dummy:=true',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[CONTROLLERS_YAML],
        output='both',
        remappings=[('~/robot_description', '/robot_description')],
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both',
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # --param-file is required from ros2_control 6.x on: the controller node no
        # longer inherits the parameter file given to ros2_control_node, so without
        # it the controller loads with empty parameters. humble and jazzy accept the
        # flag too and are unaffected, so there is one form for every distro.
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            CONTROLLERS_YAML,
        ],
        output='both',
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            CONTROLLERS_YAML,
        ],
        output='both',
    )
    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDummyBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_dummy_bringup')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_a_node_running(self):
        # 15 s rather than the 5 s default: this is the first thing checked
        # after ReadyToTest(), so it absorbs the whole ros2_control_node
        # startup on a cold, contended CI runner.
        check_node_running(self.node, 'controller_manager', timeout=15.0)

    def test_b_controllers_running(self):
        check_controllers_running(
            self.node, ['joint_state_broadcaster', 'joint_trajectory_controller'])

    def test_c_joint_states_published(self):
        check_if_js_published('/joint_states', ['joint1', 'joint2'])

    def test_d_trajectory_convergence(self):
        # DummyDriver reflects position commands into state, so a JTC goal
        # must show up on /joint_states while the trajectory runs. The poll
        # below is satisfied as soon as the spline comes within 0.05 rad of
        # the target, which happens before time_from_start elapses -- it does
        # not wait for the trajectory to finish.
        target = {'joint1': 0.5, 'joint2': -0.5}
        publisher = self.node.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 1)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(target.keys())
        point = JointTrajectoryPoint()
        point.positions = list(target.values())
        point.time_from_start = Duration(sec=2)
        trajectory.points.append(point)

        latest = {}

        def joint_states_callback(msg):
            for name, position in zip(msg.name, msg.position):
                latest[name] = position

        subscription = self.node.create_subscription(
            JointState, '/joint_states', joint_states_callback, 10)
        try:
            deadline = time.monotonic() + 10.0
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertGreater(
                publisher.get_subscription_count(), 0,
                'joint_trajectory_controller never subscribed to the trajectory topic')
            # Known failure mode, only reachable on back-to-back local runs
            # (e.g. `ctest --repeat`): run_test_isolated.py takes its domain
            # from domain_coordinator, which releases the ID on exit and hands
            # the same one straight back, so leftover discovery state from the
            # previous run can satisfy get_subscription_count() from an
            # already-dead endpoint. This single publish then goes nowhere and
            # the poll below expires against a perfectly healthy bringup --
            # controllers active, /joint_states flowing, joints simply at 0.0.
            # CI runs this once per job and never recycles a domain, so it is
            # not exposed. The known remedy, deliberately NOT applied, is to
            # re-publish inside the poll; it costs a behavioural change, since
            # the controller replaces its active trajectory on every message.
            publisher.publish(trajectory)

            deadline = time.monotonic() + 30.0
            converged = False
            while time.monotonic() < deadline and not converged:
                rclpy.spin_once(self.node, timeout_sec=0.25)
                converged = all(
                    abs(latest.get(name, float('inf')) - goal) < 0.05
                    for name, goal in target.items())
            self.assertTrue(
                converged,
                f'joint states did not converge to {target} within 30 s; '
                f'last seen: {latest}')
        finally:
            self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)
