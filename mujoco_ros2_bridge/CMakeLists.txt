cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(controller_manager REQUIRED)
find_package(glfw3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_ros2_bridge_msgs REQUIRED)
find_package(mujoco_vendor QUIET)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Threads REQUIRED)

if(mujoco_vendor_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR "")
else()
  set(MUJOCO_LOCAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco")
  find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    PATHS "${MUJOCO_LOCAL_ROOT}/include" /usr/local/include /usr/include
    NO_DEFAULT_PATH
  )
  find_library(MUJOCO_LIB
    NAMES mujoco libmujoco.so.3.9.0
    PATHS "${MUJOCO_LOCAL_ROOT}/build/lib" /usr/local/lib /usr/lib
    NO_DEFAULT_PATH
  )
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
    message(FATAL_ERROR "mujoco_vendor was not found and local MuJoCo could not be resolved.")
  endif()
  set(MUJOCO_RUNTIME_LIBRARY_DIR "${MUJOCO_LOCAL_ROOT}/build/lib")
endif()

add_library(lodepng STATIC
  third_party/lodepng/lodepng.cpp
)
target_include_directories(lodepng PRIVATE
  third_party/lodepng
)
set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_platform_ui_adapter OBJECT
  third_party/mujoco_simulate/glfw_adapter.cc
  third_party/mujoco_simulate/glfw_dispatch.cc
  third_party/mujoco_simulate/platform_ui_adapter.cc
)
target_include_directories(mujoco_platform_ui_adapter PRIVATE
  third_party/mujoco_simulate
  ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(mujoco_platform_ui_adapter PRIVATE
  glfw
  ${MUJOCO_LIB}
)
set_target_properties(mujoco_platform_ui_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(mujoco_official_simulate STATIC
  $<TARGET_OBJECTS:mujoco_platform_ui_adapter>
  third_party/mujoco_simulate/simulate.cc
)
target_include_directories(mujoco_official_simulate PRIVATE
  third_party/mujoco_simulate
  third_party/lodepng
  ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(mujoco_official_simulate PRIVATE
  glfw
  lodepng
  ${MUJOCO_LIB}
  OpenGL::OpenGL
)
set_target_properties(mujoco_official_simulate PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
  src/mujoco_system_interface.cpp
  src/mujoco_simulation.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  third_party/mujoco_simulate
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${MUJOCO_LIB}
  ${mujoco_ros2_bridge_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
  Threads::Threads
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  mujoco_official_simulate
  glfw
)
if(MUJOCO_RUNTIME_LIBRARY_DIR)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
  )
endif()
pluginlib_export_plugin_description_file(hardware_interface mujoco_ros2_bridge_plugin.xml)

add_executable(mujoco_ros2_bridge_node
  src/mujoco_ros2_bridge_node.cpp
)
target_link_libraries(mujoco_ros2_bridge_node
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

if(BUILD_TESTING)
  ament_add_gtest(test_render_mode test/test_render_mode.cpp)
  if(TARGET test_render_mode)
    target_link_libraries(test_render_mode ${PROJECT_NAME})
    if(MUJOCO_RUNTIME_LIBRARY_DIR)
      set_target_properties(test_render_mode PROPERTIES
        BUILD_RPATH "${MUJOCO_RUNTIME_LIBRARY_DIR}"
      )
    endif()
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  mujoco_ros2_bridge_msgs
  mujoco_vendor
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rosgraph_msgs
)
ament_package()
