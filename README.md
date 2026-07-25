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

## Hardware parameters

Every parameter below is a `<param>` element inside the `<ros2_control>` tag of the URDF. These tables are the reference; the OpenManipulator-X walkthrough further down shows the same parameters in context.

Parameters of the `<hardware>` tag:

| Parameter | Default | Description |
| --- | --- | --- |
| `port_name` | required | Serial port, e.g. `/dev/ttyUSB0`. The old name `usb_port` is still accepted but deprecated and logs a warning at startup. Only optional under `use_dummy`. |
| `baud_rate` | required | Serial baud rate, e.g. `1000000`. Must be at least `1`. Only optional under `use_dummy`. |
| `use_dummy` | `false` | `true` runs the built-in dummy driver, which emulates every operating mode, instead of opening the serial port. |
| `torque_enable` | `true` | `false` never turns servo torque on: activation and mode switches skip it and the joints stay limp. Meant for a leader arm in teleoperation, which is back-driven by hand while its states are published. Turning torque *off* is never skipped, so deactivation still de-energizes. |
| `read_error_tolerance` | `5` | Number of consecutive read failures at which `read()` reports an error. Must be at least `1`. |
| `write_error_tolerance` | `5` | Number of consecutive driver write failures at which `write()` reports an error. Must be at least `1`. This is a budget of its own, counted separately from `read_error_tolerance`. |

Per-joint parameters, inside each `<joint>` tag:

| Parameter | Default | Description |
| --- | --- | --- |
| `id` | required | Dynamixel servo id. Two joints may not share one id; that is rejected at initialization, naming both joints and the id they collide on. |
| `control_mode` | `position` | Operating mode applied at configuration time: one of `position`, `extended_position`, `multi_turn`, `current_based_position`, `velocity`, `current`, `torque`, `pwm`. See [Configure the operating mode per joint](#configure-the-operating-mode-per-joint). |
| `torque_constant` | unset | Motor torque constant in Nm/A; must be positive. When set, the `effort` interfaces of that joint are in Nm instead of the servo's mA. |
| `gear_ratio` | `1.0` | Motor revolutions per joint revolution. See [Gearing and offsets](#gearing-and-offsets). |
| `offset` | `0.0` | Joint-side position offset in radians. See [Gearing and offsets](#gearing-and-offsets). |
| `Profile_Velocity`, `Profile_Acceleration`, `Position_P_Gain`, `Position_I_Gain`, `Position_D_Gain`, `Velocity_P_Gain`, `Velocity_I_Gain`, `Return_Delay_Time` | unset | Integers written verbatim to that servo's control table on configure, and again after every mode change -- these are RAM registers and a mode change resets them. |

### Gearing and offsets

`gear_ratio` is the number of motor revolutions per joint revolution. Going from the motor side to the joint side, positions and velocities are divided by it and efforts are multiplied by it; commands are converted the other way around. A negative value is legal and inverts the joint's direction, which is the easy fix for a servo mounted the wrong way round. Zero and non-finite values (`nan`, `inf`) are rejected at initialization, since neither has a meaningful conversion.

`offset` shifts the joint-side position and nothing else: the reported position is `raw - offset`, where `raw` is the position after the gear conversion, and a position command has the offset added back before it is scaled to the motor side. Velocity, effort and PWM are unaffected. Any finite value is accepted, including negatives and the default `0.0`, which is a no-op.

The `pwm` command interface is never converted by `gear_ratio` or `offset`: a duty ratio is not a physical joint quantity.

### Error tolerance and the first read

Both directions of the bus absorb a burst of transient failures before reporting a fault, and each keeps its own counter.

- A failed read holds the last-known joint states and still returns success; only the `read_error_tolerance`-th consecutive failure makes `read()` report an error. Any successful read resets the counter.
- A failed driver write likewise returns success until the `write_error_tolerance`-th consecutive failure, which makes `write()` report an error. Any successful write resets the counter.

`write()` sends nothing at all until the first successful read after activation, and that first successful read also re-synchronizes the commands to the measured state. A flaky bus therefore cannot make the plugin sync-write a zero or NaN goal to servos it has just energized. Activation itself now succeeds even when its initial read fails -- it only logs a warning -- because that write guard is what keeps the bus safe until a real state arrives.

A command that is not finite is refused by the serial driver before anything is converted or sent: if any element of a batch is NaN or infinite, the whole batch is dropped rather than partially written, and the error names the offending id and value. A non-finite command is always a bug in the caller. The refusal counts against `write_error_tolerance` like any other write failure.

### Read latency and Return_Delay_Time

If your control loop misses its update rate (see [#90](https://github.com/dynamixel-community/dynamixel_hardware/issues/90)), two latency sources dominate:

- Every Dynamixel servo waits `Return_Delay_Time` × 2 µs before answering; the factory default of 250 adds 500 µs per servo per cycle. Set `<param name="Return_Delay_Time">0</param>` on every joint and the plugin writes it to the control table for you.
- USB serial adapters buffer replies for their latency-timer interval, 16 ms by default on FTDI. Reduce it to 1 ms with `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.

With both applied, a 4-5 servo bus at 1-4 Mbaud comfortably sustains a 100 Hz `controller_manager` loop.

## Demo with real ROBOTIS OpenManipulator-X

### Configure Dynamixel motor parameters

Update the `port_name` and `baud_rate` parameters, and the per-joint `id` parameters, on [`open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_hardware_examples/blob/main/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real OpenManipulator-X.

Every joint carries its own `id` inside its `<joint>` tag; see [Hardware parameters](#hardware-parameters) for the full list.

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

Each joint accepts several optional parameters next to its `id` (see [Hardware parameters](#hardware-parameters) for all of them); the two that shape its behavior the most are `control_mode` and `torque_constant`:

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
| an interface name this plugin does not know | logs a warning once and falls back to the same historical position/velocity behavior, so the controller still starts |
| any other combination of the four known interfaces | rejected, so the controller switch fails instead of the joint moving unexpectedly |

Switching a mode requires torque to be disabled on the Dynamixel, so the plugin disables torque, rewrites the operating mode and the extra control-table parameters, and then re-enables torque -- but only on the joints it is switching, and among those only the ones whose torque it had confirmed on when the switch began. Torque is tracked per joint, and a joint counts as energized only once the driver has acknowledged the torque-on; a torque-on the servo rejects leaves that joint recorded as de-energized. Joints the switch does not touch are left alone entirely. Commands are reset to the measured position at that moment, which means the first cycle after a switch sends the reset command rather than the controller's.

If a switch fails halfway -- a serial timeout, or a servo model that cannot execute the requested mode -- the affected joints are left de-energized and the plugin reports an error from every following `write()`. Re-activating the hardware component is what clears that fault; a later mode switch will not, because restoring torque is limited to the joints that were energized when that switch began. Both switch paths latch the fault, the legacy position + velocity heuristic in the table above included, so it keeps being reported instead of surfacing for a single cycle and then going quiet. `controller_manager` only logs a failed switch and starts the controller anyway, and that persistent error is what stops a limp arm from being reported as healthy. Under `torque_enable=false` the plugin never energizes anything, so de-energized is the intended state there and a switch that simply succeeds does clear the fault.

In addition to `position`, `velocity` and `effort`, every joint exports a custom `pwm` command interface for direct duty-ratio control in `[-1, 1]`. Declare it in the URDF (`<command_interface name="pwm"/>`) to use it. Under `use_dummy` the emulated servo mirrors the commanded duty ratio into its effort state, so a joint that combines `pwm` with `torque_constant` or a non-unity `gear_ratio` publishes that duty ratio scaled as if it were a current and the resulting value is meaningless; real hardware is unaffected. The duty ratio the plugin *sends* is never scaled either way.

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
