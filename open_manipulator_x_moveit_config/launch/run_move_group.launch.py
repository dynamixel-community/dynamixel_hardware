# Copyright 2021 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("open_manipulator_x_description"),
                              "urdf", "open_manipulator_x.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "open_manipulator_x_moveit_config", "config/open_manipulator_x.srdf")
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = load_yaml(
        "open_manipulator_x_moveit_config", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization \
                             default_planner_request_adapters/FixWorkspaceBounds \
                             default_planner_request_adapters/FixStartStateBounds \
                             default_planner_request_adapters/FixStartStateCollision \
                             default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1}}
    ompl_planning_yaml = load_yaml(
        "open_manipulator_x_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(
        "open_manipulator_x_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"}

    trajectory_execution = {"moveit_manage_controllers": True,
                            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                            "trajectory_execution.allowed_goal_duration_margin": 0.5,
                            "trajectory_execution.allowed_start_tolerance": 0.1}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}

    return LaunchDescription([
        Node(package="moveit_ros_move_group",
             executable="move_group",
             output="screen",
             parameters=[robot_description,
                         robot_description_semantic,
                         kinematics_yaml,
                         ompl_planning_pipeline_config,
                         trajectory_execution,
                         moveit_controllers,
                         planning_scene_monitor_parameters]),
    ])
