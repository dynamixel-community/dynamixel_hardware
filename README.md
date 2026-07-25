# dynamixel_hardware

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

The `dynamixel_hardware` package is the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.

It is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

## Set up

First [install ROS 2 Rolling on Ubuntu 22.04](http://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html). Then follow the instruction below.

```shell
$ source /opt/ros/rolling/setup.bash
$ mkdir -p ~/ros/rolling && cd ~/ros/rolling/src
$ git clone https://github.com/youtalk/dynamixel_hardware.git
$ git clone https://github.com/youtalk/dynamixel_hardware_examples.git
$ cd -
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
$ . install/setup.bash
```

## Demo with real ROBOTIS OpenManipulator-X

### Configure Dynamixel motor parameters

Update the `port_name`, `baud_rate`, and `joint_ids` parameters on [`open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_hardware_examples/blob/main/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real OpenManipulator-X.

Note that `joint_ids` parameters must be splited by `,`.

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="port_name">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <!-- <param name="use_dummy">true</param> -->
</hardware>
```

The `port_name` parameter used to be named `usb_port`. `usb_port` still works, but it is deprecated and logs a warning at startup; rename it to `port_name` in your URDF.

Deactivating the hardware component (or shutting down the `controller_manager`) now disables torque, so the joints go limp -- previously `on_deactivate` was a no-op and the servos stayed energized. Keep this in mind before deactivating a robot that isn't resting in a safe pose.

### Configure the operating mode per joint

Each joint accepts two optional parameters next to its `id`:

```xml
<joint name="joint1">
  <param name="id">11</param>
  <param name="control_mode">current_based_position</param>
  <param name="torque_constant">1.79</param>
  <command_interface name="position"/>
  <command_interface name="effort"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

`control_mode` selects the Dynamixel operating mode the joint is put into at configuration time. It defaults to `position` and accepts `position`, `extended_position`, `multi_turn`, `current_based_position`, `velocity`, `current`, `torque` and `pwm`. An unknown value fails the lifecycle transition, and so does a mode the servo model cannot execute -- the failure names the joint id and the model, so an unsupported combination is reported before any controller starts instead of silently doing nothing.

`torque_constant` is the motor torque constant in Nm/A and must be positive. When it is set, the `effort` command and state interfaces of that joint are in Nm and are converted to and from the servo's milliamps for you. When it is omitted, the `effort` interfaces carry the raw current in mA.

The mode a joint actually runs in follows the command interfaces the active controller claims, so different joints can run in different modes in the same control cycle:

| Claimed command interfaces | Operating mode |
| --- | --- |
| `position` | the configured mode when it is one of the position-family modes, otherwise `position` |
| `velocity` | `velocity` |
| `effort` | `current`, or `torque` when the joint is configured as `torque` |
| `pwm` | `pwm` |
| `position` + `effort` | `current_based_position` (the effort command is the current limit) |
| `position` + `velocity` | kept on the historical behavior: a changed velocity command switches the joint to velocity control, otherwise a changed position command switches it back to position control |
| anything else | rejected, so the controller switch fails instead of the joint moving unexpectedly |

Switching a mode requires torque to be disabled on the Dynamixel, so the plugin disables torque, rewrites the operating mode and the extra control-table parameters, and re-enables torque for exactly the joints that change. Commands are reset to the measured position at that moment, which means the first cycle after a switch sends the reset command rather than the controller's.

In addition to `position`, `velocity` and `effort`, every joint exports a custom `pwm` command interface for direct duty-ratio control in `[-1, 1]`. Declare it in the URDF (`<command_interface name="pwm"/>`) to use it.

- Terminal 1

Launch the `ros2_control` manager for the OpenManipulator-X.

```shell
$ ros2 launch open_manipulator_x_description open_manipulator_x.launch.py
```

- Terminal 2

Start the `joint_trajectory_controller` and send a `/joint_trajectory_controller/follow_joint_trajectory` goal to move the OpenManipulator-X.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --activate joint_trajectory_controller --deactivate velocity_controller
$ ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, gripper],
    points: [
      { positions: [0.1, 0.1, 0.1, 0.1, 0], time_from_start: { sec: 2 } },
      { positions: [-0.1, -0.1, -0.1, -0.1, 0], time_from_start: { sec: 4 } },
      { positions: [0, 0, 0, 0, 0], time_from_start: { sec: 6 } }
    ]
  }
}"
```

If you would like to use the velocity control instead, switch to the `velocity_controller` and publish a `/velocity_controller/commands` message to move the OpenManipulator-X.

```shell
$ ros2 control switch_controllers --activate joint_state_broadcaster --deactivate joint_trajectory_controller --activate velocity_controller
$ ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.1, 0]"
```

[![dynamixel_control: the ros2_control implementation for any kind of ROBOTIS Dynamixel robots](https://img.youtube.com/vi/EZtBaU-otzI/0.jpg)](https://www.youtube.com/watch?v=EZtBaU-otzI)

## Demo with dummy ROBOTIS OpenManipulator-X

The `use_dummy` parameter is required if you use the dummy OpenManipulator-X.

```diff
diff --git a/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro b/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
index c6cdb74..111846d 100644
--- a/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
+++ b/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro
@@ -9,7 +9,7 @@
         <param name="usb_port">/dev/ttyUSB0</param>
         <param name="baud_rate">1000000</param>
-        <!-- <param name="use_dummy">true</param> -->
+        <param name="use_dummy">true</param>
       </hardware>
       <joint name="joint1">
         <param name="id">11</param>
```

Then follow the same instruction of the real robot one.

Note that the dummy implementation has no interpolation so far.
If you sent a joint message, the robot would move directly to the joints without interpolation.
