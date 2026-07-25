cmake_minimum_required(VERSION 3.16)
project(dynamixel_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

# hardware_interface >= 6.7.1 (lyrical/rolling) requires C++20; C++17 otherwise.
if(NOT CMAKE_CXX_STANDARD)
  if(hardware_interface_VERSION VERSION_GREATER_EQUAL 6.7.1)
    set(CMAKE_CXX_STANDARD 20)
  else()
    set(CMAKE_CXX_STANDARD 17)
  endif()
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} SHARED
  src/dynamixel_hardware.cpp
  src/dummy_driver.cpp
  src/workbench_driver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${lifecycle_msgs_TARGETS}
  dynamixel_workbench_toolbox::dynamixel_workbench_toolbox
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_dummy_driver test/test_dummy_driver.cpp)
  target_include_directories(test_dummy_driver PRIVATE include)
  target_link_libraries(test_dummy_driver ${PROJECT_NAME})

  ament_add_gmock(test_load_dynamixel_hardware test/test_load_dynamixel_hardware.cpp)
  target_include_directories(test_load_dynamixel_hardware PRIVATE include)
  target_link_libraries(test_load_dynamixel_hardware
    ${PROJECT_NAME}
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_workbench_driver test/test_workbench_driver.cpp)
  target_include_directories(test_workbench_driver PRIVATE include)
  target_link_libraries(test_workbench_driver
    ${PROJECT_NAME}
    dynamixel_workbench_toolbox::dynamixel_workbench_toolbox
  )

  ament_add_gmock(test_dynamixel_hardware test/test_dynamixel_hardware.cpp)
  target_include_directories(test_dynamixel_hardware PRIVATE include test)
  target_link_libraries(test_dynamixel_hardware
    ${PROJECT_NAME}
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ros2_control_test_assets::ros2_control_test_assets
  )

  # controller_manager has no installable rolling binaries during the
  # Ubuntu-resolute transition (released in rosdistro, buildfarm not yet
  # synced) — register the launch test only where it can actually run.
  # Everything the launch test needs, including the REQUIRED find_package
  # calls, lives inside the guard: the same incomplete rolling dependency
  # set that makes controller_manager unavailable can just as well drop
  # these, and a REQUIRED lookup out here would turn the graceful skip into
  # a hard build failure on the one distro the guard exists for.
  find_package(controller_manager QUIET)
  if(controller_manager_FOUND)
    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    function(add_ros_isolated_launch_test path)
      set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
      add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
    endfunction()

    # TIMEOUT budget. unittest runs all four sub-tests regardless of whether
    # an earlier one failed, so the worst case is the SUM of their deadlines,
    # not the largest of them:
    #
    #   test_a_node_running            15 s  (check_node_running timeout)
    #   test_b_controllers_running  20-25 s  (two 10 s loops; the second makes
    #                                         a 5 s list_controllers call that
    #                                         can overrun its own deadline)
    #   test_c_joint_states_published  20 s  (WaitForTopics, fixed in
    #                                         controller_manager.test_utils)
    #   test_d_trajectory_convergence  40 s  (10 s subscriber wait + 30 s poll)
    #                                -------
    #                                95-100 s, plus launch startup and SIGINT
    #                                teardown -> ~100-105 s
    #
    # A passing run takes ~3 s, so this is not about the happy path: exceeding
    # the budget makes ctest report a bare "Timeout" and throws away every
    # assertion message the test exists to produce. 180 keeps a real margin on
    # a cold, contended runner. Re-derive this sum before tightening it or
    # adding a fifth sub-test.
    add_ros_isolated_launch_test(test/launch/test_dummy_bringup.launch.py TIMEOUT 180)
  else()
    # Announced so the skip is greppable in the build log. Without this a
    # dependency regression on a distro that is supposed to run the launch
    # test would silently unregister it and leave CI green with the
    # integration coverage gone. STATUS rather than WARNING on purpose:
    # WARNING would give every rolling build a permanent '--- stderr' block,
    # and a build that always warns is a build nobody reads.
    message(STATUS
      "controller_manager not found - skipping the dummy bringup launch test")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  hardware_interface
  pluginlib
  dynamixel_workbench_toolbox
)

ament_package()
