# dynamixel_control

## Set up

```shell
$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/ros/foxy && cd ~/ros/foxy
$ git clone https://github.com/youtalk/dynamixel_control.git src
$ vcs import src < src/dynamixel_control.repos
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
$ . install/setup.bash
```

## Demo with real ROBOTIS OpenManipulator-X

- Terminal 1

```shell
$ ros2 launch open_manipulator_x_robot open_manipulator_x_robot.launch.py
```

- Terminal 2

```shell
$ ros2 control load_start_controller joint_state_controller
$ ros2 control load_configure_controller forward_command_controller_position
$ ros2 control switch_controllers --start-controllers forward_command_controller_position
$ ros2 control list_controllers
$ ros2 control list_hardware_interfaces
$ ros2 topic pub /forward_command_controller_position/commands std_msgs/msg/Float64MultiArray "data:
- 0.1
- 0.1
- 0.1
- 0.1
- 0.01"
```

## Demo with dummy ROBOTIS OpenManipulator-X
