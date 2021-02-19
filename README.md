# dynamixel_control

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

- `dynamixel_hardware`: the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.
- `open_manipulator_x_robot`: the reference implementation of the `ros2_control` robot using [ROBOTIS OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/).

The `dynamixel_hardware` package is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

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

### Configure Dynamixel motor parameters

First update the `usb_port`, `baud_rate`, and `joint_ids` parameters on [`open_manipulator_x_robot/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_control/blob/main/open_manipulator_x_robot/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real OpenManipulator-X.

Note that `joint_ids` parameters must be splited by `,`.

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="usb_port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <param name="joint_ids">11,12,13,14,15</param>
  <!-- <param name="use_dummy">true</param> -->
</hardware>
```

- Terminal 1

Launch the `ros2_control` manager for the OpenManipulator-X.

```shell
$ ros2 launch open_manipulator_x_robot open_manipulator_x_robot.launch.py
```

- Terminal 2

Load the `joint_state_controller` and `forward_command_controller_position`.
Next start the `forward_command_controller_position` and send a `/forward_command_controller_position/commands` message to move the OpenManipulator-X.

```shell
$ ros2 control load_start_controller joint_state_controller
$ ros2 control load_configure_controller forward_command_controller_position
$ ros2 control switch_controllers --start-controllers forward_command_controller_position
$ ros2 control list_controllers
$ ros2 control list_hardware_interfaces
$ ros2 topic pub /forward_command_controller_position/commands std_msgs/msg/Float64MultiArray "data: [0.1, 0.1, 0.1, 0.1, 0.01]"
```

## Demo with dummy ROBOTIS OpenManipulator-X

```diff
diff --git a/open_manipulator_x_robot/urdf/open_manipulator_x.ros2_control.xacro b/open_manipulator_x_robot/urdf/open_manipulator_x.ros2_control.xacro
index c6cdb74..111846d 100644
--- a/open_manipulator_x_robot/urdf/open_manipulator_x.ros2_control.xacro
+++ b/open_manipulator_x_robot/urdf/open_manipulator_x.ros2_control.xacro
@@ -9,7 +9,7 @@
         <param name="usb_port">/dev/ttyUSB0</param>
         <param name="baud_rate">1000000</param>
         <param name="joint_ids">11,12,13,14,15</param>
-        <!-- <param name="use_dummy">true</param> -->
+        <param name="use_dummy">true</param>
       </hardware>
       <joint name="joint1">
         <command_interface name="position"/>
```
