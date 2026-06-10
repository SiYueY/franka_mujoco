#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "franka_hardware/data.hpp"
#include "mujoco_simulation/hardware/camera.hpp"
#include "mujoco_simulation/hardware/imu.hpp"
#include "mujoco_simulation/hardware/lidar.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace franka_hardware {

class SensorBridge {
 public:
  SensorBridge(const std::string& node_name, const std::vector<ImuData>* imus,
               const std::vector<CameraData>* cameras, const std::vector<LidarData>* lidars);

  void set_time(const rclcpp::Time& sim_time);
  bool publish_imu(const ImuData& imu);
  bool publish_camera(const CameraData& camera);
  bool publish_lidar(const LidarData& lidar);

 private:
  struct ImuPublisher {
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  };

  struct CameraPublisher {
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info;
  };

  struct LidarPublisher {
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  };

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Time sim_time_{0, 0, RCL_ROS_TIME};
  std::unordered_map<std::string, ImuPublisher> imus_;
  std::unordered_map<std::string, CameraPublisher> cameras_;
  std::unordered_map<std::string, LidarPublisher> lidars_;
};

}  // namespace franka_hardware
