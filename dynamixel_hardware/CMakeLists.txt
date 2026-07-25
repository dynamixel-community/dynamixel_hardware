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
