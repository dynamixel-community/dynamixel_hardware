# Migrating to `dynamixel_hardware_interface`

`dynamixel_hardware` is deprecated. This guide moves a working `dynamixel_hardware` setup to [ROBOTIS-GIT/dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface), the `ros2_control` hardware interface maintained by ROBOTIS.

## 1. Why this package is deprecated

ROBOTIS maintains an official `ros2_control` `SystemInterface` for Dynamixel servos. It fills the same role as this package, is released for every ROS 2 distro this package targets, and is the hardware layer behind ROBOTIS's own OpenManipulator-X, OMY, and AI Worker products.

It also does a number of things this package never did — see [section 4](#4-what-you-gain). Maintaining a second, less capable implementation of the same thing does not serve anyone, so this package is being retired rather than developed further.

If you are here because you were hoping for better bus responsiveness: `dynamixel_workbench_toolbox`, which this package uses, is itself a thin wrapper over `dynamixel_sdk`, so the wire protocol is identical between the two packages. The real difference is that the official package uses Fast Sync Read, which aggregates every servo's reply into one packet. The gain is small at five joints and grows with servo count.

## 2. Installing the official package

Binaries are available for every distro:

```bash
sudo apt install ros-$ROS_DISTRO-dynamixel-hardware-interface
```

| Distro | `dynamixel_hardware_interface` |
| --- | --- |
| humble | 1.5.0-2 |
| jazzy | 1.5.2-1 |
| kilted | 1.5.1-1 |
| lyrical | 1.5.1-3 |
| rolling | 1.5.1-2 |

Its `dynamixel_sdk` and `dynamixel_interfaces` dependencies are released for all five as well.

To build from source instead, clone the three repositories on the branch matching your distro:

```bash
cd ~/${WORKSPACE}/src
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
```

## 3. Converting your URDF

The two packages describe a servo differently.

`dynamixel_hardware` puts the servo id and every per-servo setting inside the `<joint>` tag. `dynamixel_hardware_interface` keeps `<joint>` for the `ros2_control`-facing interfaces only, and adds a **parallel `<gpio>` block per servo** that declares the physical Dynamixel and names the control-table items to sync-read and sync-write. **Both blocks are required.**

### 3.1 A complete before and after

Before — the minimal example from this package's README:

```xml
<ros2_control name="DynamixelHardware" type="system">
  <hardware>
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="port_name">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>
  </hardware>
  <joint name="joint1">
    <param name="id">1</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

After:

```xml
<ros2_control name="DynamixelHardware" type="system">
  <hardware>
    <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
    <param name="port_name">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>
    <param name="error_timeout_ms">500</param>
    <param name="dynamixel_model_folder">/param/dxl_model</param>
    <param name="number_of_joints">1</param>
    <param name="number_of_transmissions">1</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <gpio name="dxl1">
    <param name="type">dxl</param>
    <param name="ID">1</param>
    <param name="Operating Mode">3</param>
    <command_interface name="Goal Position"/>
    <state_interface name="Present Position"/>
    <state_interface name="Present Velocity"/>
    <state_interface name="Present Current"/>
  </gpio>
</ros2_control>
```

Note what moved: the plugin name, the servo `id` (now `ID`, in the `<gpio>` block), and the addition of three mandatory hardware parameters plus the `<gpio>` block itself. `Operating Mode` `3` is Position; the full set is 0 Current, 1 Velocity, 3 Position, 4 Extended Position, 5 Current-based Position, 16 PWM.

For a full multi-joint example including a gripper, read ROBOTIS's own [`omx_f.ros2_control.xacro`](https://github.com/ROBOTIS-GIT/open_manipulator/blob/main/open_manipulator_description/ros2_control/omx_f.ros2_control.xacro).

### 3.2 Hardware parameters

Parameters of the `<hardware>` tag:

| `dynamixel_hardware` | `dynamixel_hardware_interface` | Notes |
| --- | --- | --- |
| `port_name` | `port_name` | Identical. If you still use the deprecated `usb_port` spelling, rename it. |
| `baud_rate` | `baud_rate` | Identical. |
| `torque_enable` (default `true`) | `disable_torque_at_init` (default `false`) | Inverted sense. `torque_enable=false` becomes `disable_torque_at_init=true`. |
| `read_error_tolerance`, `write_error_tolerance` (counts, default `5` each) | `error_timeout_ms` (elapsed ms, default `500`) | Two separate budgets collapse into one, and the unit changes from consecutive failures to elapsed error time. The official package also attempts an automatic port reset and re-initialisation rather than reporting an error to the controller manager. |
| `use_dummy` | — | No equivalent. See [section 5](#5-what-you-lose). |
| — | `number_of_joints` | **Required.** |
| — | `number_of_transmissions` | **Required.** Set equal to `number_of_joints` when each joint is driven by exactly one servo. |
| — | `dynamixel_model_folder` | **Required.** Appended to `dynamixel_hardware_interface`'s own share directory, so the value is effectively always `/param/dxl_model`. |
| — | `transmission_to_joint_matrix`, `joint_to_transmission_matrix` | Optional, comma-separated, row-major, `number_of_joints × number_of_transmissions` elements. Default to the identity matrix when omitted. |
| — | `use_revolute_to_prismatic_gripper`, `revolute_to_prismatic_dxl`, `revolute_to_prismatic_joint`, `prismatic_min`, `prismatic_max`, `revolute_min`, `revolute_max` | Optional. Maps a rotary gripper servo onto a prismatic finger-travel joint. |
| — | `dynamixel_state_pub_msg_name`, `get_dynamixel_data_srv_name`, `set_dynamixel_data_srv_name`, `reboot_dxl_srv_name`, `set_dxl_torque_srv_name` | Optional topic and service name overrides. |

### 3.3 Per-servo parameters

These move from `<joint>` to the servo's `<gpio>` block:

| `dynamixel_hardware` (`<joint>`) | `dynamixel_hardware_interface` (`<gpio>`) | Notes |
| --- | --- | --- |
| `id` | `ID` | Capitalisation differs. |
| `control_mode` (`position`, `velocity`, `current`, …) | `Operating Mode` (integer) | `position`→`3`, `extended_position`→`4`, `current_based_position`→`5`, `velocity`→`1`, `current`→`0`, `pwm`→`16`. Written to EEPROM once at init and **not switchable at runtime** — see [section 5](#5-what-you-lose). `multi_turn` and `torque` are Protocol 1.0 modes with no counterpart. |
| `gear_ratio` | `transmission_to_joint_matrix` / `joint_to_transmission_matrix` | A scalar ratio *r* becomes the diagonal element `1/r` of the transmission-to-joint matrix and `r` of the joint-to-transmission matrix. These are hardware-level parameters, not per-servo ones. |
| `offset` | `[unit info]` override on `Present Position` and `Goal Position` | See [section 3.5](#35-unit-overrides). |
| `torque_constant` | `[unit info]` override on `Present Current` and `Goal Current` | See [section 3.5](#35-unit-overrides). |
| `Profile_Velocity`, `Profile_Acceleration`, `Position_P_Gain`, `Position_I_Gain`, `Position_D_Gain`, `Velocity_P_Gain`, `Velocity_I_Gain`, `Return_Delay_Time` | `Profile Velocity`, `Profile Acceleration`, `Position P Gain`, `Position I Gain`, `Position D Gain`, `Velocity P Gain`, `Velocity I Gain`, `Return Delay Time` | Underscores become spaces. Unlike this package's fixed list of eight, the official package writes **any** parameter whose name matches a control-table item, so `Drive Mode`, the various `Limit` registers, and everything else in the table are available too. |
| — | `type` | **Required.** `dxl` for a servo. Other values: `virtual_dxl`, `sensor`, `controller`, `virtual_sensor`. |
| — | `Reboot` | Optional. `1` reboots that servo during initialisation. |

### 3.4 Interfaces

| `dynamixel_hardware` | `dynamixel_hardware_interface` |
| --- | --- |
| `<joint>` command `position`, `velocity`, `effort`, `pwm` | `<joint>` command `position`, `velocity`, `effort`, `acceleration`. There is no joint-level `pwm`; drive PWM through a raw `<gpio><command_interface name="Goal PWM"/>`. |
| `<joint>` state `position`, `velocity`, `effort` | Same, plus `acceleration`, `hardware_state`, and `torque_enable`. |
| — | `<gpio>` `<command_interface>` and `<state_interface>` naming raw control-table items. **Required** — these are what actually gets sync-written and sync-read. `Goal Position` backs the joint's `position` command, `Present Position`/`Present Velocity` back its `position`/`velocity` states, and `Present Current` (or `Present Load`) backs `effort`. |

### 3.5 Unit overrides

The official package reads its scaling from the servo's `.model` file. A `[unit info]` parameter on a `<gpio>` block overrides it per servo. Each line is `<data name>, <multiplier>, <unit>, signed|unsigned, <offset>`, and the physical value is `raw × multiplier + offset`:

```xml
<gpio name="dxl1">
  <param name="type">dxl</param>
  <param name="ID">1</param>
  <param name="[unit info]">
    Present Position, 0.0015339807878856412, rad, signed, -3.14159265359
    Goal Position, 0.0015339807878856412, rad, signed, -3.14159265359
  </param>
  <command_interface name="Goal Position"/>
  <state_interface name="Present Position"/>
</gpio>
```

This is the closest equivalent to this package's `offset` (adjust the trailing offset column) and `torque_constant` (adjust the multiplier column on `Present Current` and `Goal Current`, which the official package otherwise reports raw).

> **Not verified on hardware.** These two mappings were derived by reading the official package's source, not by running them on servos. Check the resulting scaling against a known pose or a known load before trusting it.

## 4. What you gain

**Communication**

- **Fast Sync Read and Fast Bulk Read** (Protocol 2.0 instructions `0x8A` and `0x9A`), which aggregate every servo's reply into a single status packet, with automatic fallback to the ordinary versions after repeated failures.
- **Indirect-address packing**: control-table items that are far apart get mapped into one contiguous indirect block and fetched in one transfer, instead of a fixed window that has to span the gap.
- **Automatic Sync-versus-Bulk selection**, so servos with differing control tables can share a bus.
- **Multi-bus operation**: one hardware component can drive several U2D2 adapters.

**Mechanism modelling**

- **Joint-to-transmission matrices**, so one joint can be driven by several servos — differential wrists and coupled gearing.
- **Revolute-to-prismatic conversion** for gripper finger travel.

**Control-table access**

- **Any control-table item as a `ros2_control` interface**, so temperature, input voltage, or realtime tick can be published as state interfaces.
- **Any control-table item writable from the URDF**, not a fixed whitelist.
- A **110-model control-table database**, where adding a new servo model is a matter of dropping in one text file.
- **Firmware-version-specific model files**, selected automatically from the servo's reported firmware version.
- Support for **non-servo Dynamixel-protocol devices** — IMUs, LED units, control units — on the same bus.

**Operations and diagnostics**

- A **realtime `DynamixelState` publisher** carrying eighteen communication status codes plus per-servo hardware error flags: input voltage, overheating, encoder, electrical shock, and overload.
- Services to **read and write any control-table item at runtime**, to **reboot a servo**, to **toggle torque**, and to **fetch an error summary**.
- **`hardware_state` and `torque_enable` state interfaces**, and a runtime torque-enable command interface.
- **Automatic communication recovery** on error, rather than a transition to the error state.

## 5. What you lose

Stated plainly, with an honest workaround or an honest "none".

| Lost | What to do |
| --- | --- |
| **Runtime control-mode switching.** This package switched a joint's operating mode to follow whichever command interfaces the active controller claimed, through `prepare_command_mode_switch` / `perform_command_mode_switch`. | **No workaround.** The official package writes `Operating Mode` to EEPROM once during initialisation. If you switch modes at runtime, open an issue on [the official tracker](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/issues) describing your use case, or pin this package as described in [section 6](#6-if-you-cannot-migrate). |
| **`use_dummy`**, the built-in driver that emulated every operating mode with no servos attached. | Use `ros2_control`'s standard `mock_components/GenericSystem` plugin, which gives joint-state loopback without hardware. It does not emulate per-mode servo behaviour. ROBOTIS's [`omx_f.ros2_control.xacro`](https://github.com/ROBOTIS-GIT/open_manipulator/blob/main/open_manipulator_description/ros2_control/omx_f.ros2_control.xacro) shows the pattern behind a `use_mock_hardware` xacro argument. |
| **Protocol 1.0 servos** — the AX series, and MX servos on pre-2.0 firmware. This package fell back to `Moving_Speed`, `Present_Speed`, `Present_Load`, and `Goal_Torque` for them. | The official package requests the default packet handler, which is Protocol 2.0 only. **MX servos can be migrated** by updating them to firmware 2.0 with ROBOTIS's documented procedure. **AX servos cannot** — pin this package as described in [section 6](#6-if-you-cannot-migrate). |
| **`on_configure` / `on_cleanup` / `on_error` lifecycle handling.** This package opened the port in `on_configure`, so cycling the component's lifecycle state reconnected it. | The official package implements only `on_init` and `on_activate` and opens the port during `on_init`. Reconnecting by lifecycle transition is not available; it recovers internally instead. |
| **The `multi_turn` and `torque` operating modes.** | Both are Protocol 1.0 MX modes and follow the Protocol 1.0 row above. `extended_position` (`Operating Mode` 4) covers multi-turn positioning on Protocol 2.0 servos. |

## 6. If you cannot migrate

Both repositories are archived and read-only. Nothing here will receive further fixes, and no support is offered — but the code does not stop working, and you can pin it.

In a `.repos` file:

```yaml
repositories:
  dynamixel_hardware:
    type: git
    url: https://github.com/dynamixel-community/dynamixel_hardware.git
    version: rolling   # or humble, jazzy, lyrical -- all four are identical
```

Pin a commit rather than a branch if you want the reference to be immutable. The final commit on every distro branch is the one that added this guide.

The four distro branches carry identical content and the last release to the buildfarm was 0.6.1. Published binaries stay published — `apt install ros-<distro>-dynamixel-hardware` keeps working for as long as your distro's repository does — but they are older than these branches and will never be refreshed.
