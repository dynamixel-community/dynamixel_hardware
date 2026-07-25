^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add Mergify backport automation and rewrite the README for 1.0.0
* Add a launch_testing integration test for the dummy bringup (`#114 <https://github.com/dynamixel-community/dynamixel_hardware/issues/114>`_)
* Finalize the hardware parameter set and the read/write error-handling policy: port_name (usb_port deprecated), torque_enable, read_error_tolerance, write_error_tolerance, gear_ratio, offset, torque_constant, and Return_Delay_Time; hold the last known state on read errors and withhold writes until the first successful read (`#113 <https://github.com/dynamixel-community/dynamixel_hardware/issues/113>`_)
* Support all eight Dynamixel operating modes with per-joint command-mode switching and full dummy-mode emulation (`#112 <https://github.com/dynamixel-community/dynamixel_hardware/issues/112>`_)
* Refactor into a driver abstraction (DynamixelDriver / WorkbenchDriver / DummyDriver), normalize the lifecycle callbacks, and add the unit-test infrastructure (`#111 <https://github.com/dynamixel-community/dynamixel_hardware/issues/111>`_)
* Modernize the build: namespaced CMake targets, automatic C++17/C++20 standard selection, and the visibility_control.h rename (`#110 <https://github.com/dynamixel-community/dynamixel_hardware/issues/110>`_)
* Replace ros-tooling CI with official ROS container workflows for humble, jazzy, lyrical, and rolling, plus a nightly cross-branch and downstream-examples test matrix (`#109 <https://github.com/dynamixel-community/dynamixel_hardware/issues/109>`_)
* Contributors: Ignacio Davila, Kenji Brameld, Maverobot, Tacha-S, Yutaka Kondo, moyashibeans, sebtiburzio, soham2560

0.6.1 (2026-07-24)
------------------
* Link namespaced CMake targets instead of ament_target_dependencies
* Update the deprecated on_init function to use new parameter type (`#108 <https://github.com/dynamixel-community/dynamixel_hardware/issues/108>`_)
* Contributors: Yutaka Kondo, Zheng Qu

0.6.0 (2024-04-24)
------------------
* Adhere to style guide (`#73 <https://github.com/dynamixel-community/dynamixel_hardware/issues/73>`_)
* Revised control mode changes, added set_joint_params (`#65 <https://github.com/dynamixel-community/dynamixel_hardware/issues/65>`_)
  * revised control mode changes, added set_params
  * removed unnecessary comment
* Missing comma for setting Position_D_Gain (`#56 <https://github.com/dynamixel-community/dynamixel_hardware/issues/56>`_)
  * comment out unused paramter
  * A comma is missing for setting the Position_D_Gain
* Contributors: Geoff Sokoll, Kenji Brameld, Lass6230, Yutaka Kondo

0.3.1 (2022-11-17)
------------------
* Merge pull request `#39 <https://github.com/youtalk/dynamixel_control/issues/39>`_ from ijnek/ijnek-unused-parameters
* Merge pull request `#31 <https://github.com/youtalk/dynamixel_control/issues/31>`_ from ijnek/ijnek-unused-parameter-2
* Merge pull request `#25 <https://github.com/youtalk/dynamixel_control/issues/25>`_ from ijnek/ijnek-new-hardware-interface
* Merge pull request `#24 <https://github.com/youtalk/dynamixel_control/issues/24>`_ from ijnek/ijnek-add-callbackreturn-reference
* Merge pull request `#20 <https://github.com/youtalk/dynamixel_control/issues/20>`_ from pdenes/add-extra-params
* Merge pull request `#19 <https://github.com/youtalk/dynamixel_control/issues/19>`_ from pdenes/add-dependencies
* Merge pull request `#17 <https://github.com/youtalk/dynamixel_control/issues/17>`_ from youtalk/galactic
* Merge pull request `#16 <https://github.com/youtalk/dynamixel_control/issues/16>`_ from Schnilz/main
* Merge pull request `#13 <https://github.com/youtalk/dynamixel_control/issues/13>`_ from youtalk/change-loglevel
* Merge pull request `#10 <https://github.com/youtalk/dynamixel_control/issues/10>`_ from youtalk/joint-id
* Merge pull request `#1 <https://github.com/youtalk/dynamixel_control/issues/1>`_ from youtalk/velocity-control
* Contributors: Kenji Brameld, Nils Schulte, Pal Denes, Yutaka Kondo
