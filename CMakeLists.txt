cmake_minimum_required(VERSION 3.8)
project(gz_virtual_base_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

add_library(gz_virtual_base_system SHARED src/gz_virtual_base_system.cpp)

target_include_directories(gz_virtual_base_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(gz_virtual_base_system PUBLIC gz-sim::gz-sim)

ament_target_dependencies(gz_virtual_base_system PUBLIC
  gz_ros2_control hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(gz_ros2_control gz_virtual_base_system_plugin.xml)



add_library(gz_virtual_base_velocity_controller SHARED
  src/gz_virtual_base_velocity_controller.cpp
)

target_include_directories(gz_virtual_base_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(gz_virtual_base_velocity_controller
  controller_interface
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  controller_interface
  gz_virtual_base_velocity_controller_plugin.xml
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS gz_virtual_base_system gz_virtual_base_velocity_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config urdf launch world DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz_ros2_control 
  hardware_interface 
  controller_interface 
  geometry_msgs 
  pluginlib rclcpp 
  rclcpp_lifecycle
)
ament_package()
