#pragma once

#include <hardware_interface/hardware_info.hpp>
#include <memory>
#include <mujoco_ros2_bridge_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_bridge_msgs/srv/set_pause.hpp>
#include <mujoco_ros2_bridge_msgs/srv/step_simulation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>

#include "mujoco_simulation/mujoco_camera.hpp"
#include "mujoco_simulation/mujoco_lidar.hpp"
#include "mujoco_simulation/mujoco_simulation.hpp"

namespace mujoco_ros2_bridge {

struct BridgeConfig {
  bool publish_clock = true;
  bool enable_sim_services = true;
};

class MujocoRosBridge {
 public:
  MujocoRosBridge(rclcpp::Node::SharedPtr node,
                  std::shared_ptr<mujoco_simulation::MuJoCoSimulation> simulation);
  ~MujocoRosBridge();

  bool initialize(const hardware_interface::HardwareInfo& hardware_info, const BridgeConfig& config,
                  std::string* error_message);
  void start();
  void stop();

  rclcpp::Node::SharedPtr node() const { return node_; }

 private:
  void publish_clock_once();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<mujoco_simulation::MuJoCoSimulation> simulation_;
  std::unique_ptr<mujoco_simulation::MujocoCameras> cameras_;
  std::unique_ptr<mujoco_simulation::MujocoLidars> lidars_;
  BridgeConfig config_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::SetPause>::SharedPtr set_pause_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::ResetWorld>::SharedPtr reset_world_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::StepSimulation>::SharedPtr step_simulation_service_;
};

}  // namespace mujoco_ros2_bridge
