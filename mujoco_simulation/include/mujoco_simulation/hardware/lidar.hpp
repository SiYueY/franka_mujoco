#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <hardware_interface/hardware_info.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_simulation/mujoco_simulation.hpp"

namespace mujoco_simulation {

struct LidarData {
  std::string sensor_name;
  std::string sensor_prefix;
  std::string frame_name;
  std::string scan_topic;
  double publish_rate = 5.0;
  double angle_min = 0.0;
  double angle_max = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  std::vector<int> sensor_indices;
  sensor_msgs::msg::LaserScan scan_msg;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
};

class MujocoLidars {
 public:
  MujocoLidars(rclcpp::Node::SharedPtr node, MuJoCoSimulation* simulation);
  ~MujocoLidars();

  bool register_lidars(const hardware_interface::HardwareInfo& hardware_info,
                       std::string* error_message);
  void start();
  void stop();

 private:
  void update_loop();
  void update_once();

  rclcpp::Node::SharedPtr node_;
  MuJoCoSimulation* simulation_ = nullptr;
  std::vector<LidarData> lidars_;
  std::vector<mjtNum> sensor_data_;
  std::thread publish_thread_;
  std::atomic_bool publish_lidar_{false};
};

}  // namespace mujoco_simulation
