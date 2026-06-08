cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_ros2_bridge_msgs REQUIRED)
find_package(mujoco_simulation REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mujoco_context.cpp
  src/mujoco_ros_bridge.cpp
  src/mujoco_system_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  mujoco_simulation::mujoco_simulation
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${mujoco_ros2_bridge_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(hardware_interface mujoco_ros2_bridge_plugin.xml)

add_executable(mujoco_ros2_bridge_node
  src/mujoco_ros2_bridge_node.cpp
)
target_link_libraries(mujoco_ros2_bridge_node
  ${PROJECT_NAME}
  controller_manager::controller_manager
  rclcpp::rclcpp
  realtime_tools::realtime_tools
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} mujoco_ros2_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES mujoco_ros2_bridge_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  mujoco_simulation
  mujoco_ros2_bridge_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
)
ament_package()
