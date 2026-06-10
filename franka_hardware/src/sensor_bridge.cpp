#include "franka_hardware/sensor_bridge.hpp"

#include "rclcpp/qos.hpp"

namespace franka_hardware {

SensorBridge::SensorBridge(const std::string& node_name, const std::vector<ImuData>* imus,
                           const std::vector<CameraData>* cameras,
                           const std::vector<LidarData>* lidars)
    : node_(std::make_shared<rclcpp::Node>(node_name)) {
  if (imus != nullptr) {
    for (const auto& imu : *imus) {
      ImuPublisher binding;
      binding.imu =
          node_->create_publisher<sensor_msgs::msg::Imu>(imu.topic, rclcpp::SensorDataQoS());
      imus_.emplace(imu.name, std::move(binding));
    }
  }

  if (cameras != nullptr) {
    for (const auto& camera : *cameras) {
      CameraPublisher binding;
      if (camera.info.enable_rgb) {
        binding.rgb = node_->create_publisher<sensor_msgs::msg::Image>(camera.rgb_topic,
                                                                       rclcpp::SensorDataQoS());
      }
      if (camera.info.enable_depth) {
        binding.depth = node_->create_publisher<sensor_msgs::msg::Image>(camera.depth_topic,
                                                                         rclcpp::SensorDataQoS());
      }
      binding.camera_info = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          camera.camera_info_topic, rclcpp::SensorDataQoS());
      cameras_.emplace(camera.name, std::move(binding));
    }
  }

  if (lidars != nullptr) {
    for (const auto& lidar : *lidars) {
      LidarPublisher binding;
      binding.scan = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar.topic,
                                                                          rclcpp::SensorDataQoS());
      lidars_.emplace(lidar.name, std::move(binding));
    }
  }
}

void SensorBridge::set_time(const rclcpp::Time& sim_time) { sim_time_ = sim_time; }

bool SensorBridge::publish_imu(const ImuData& imu) {
  const auto it = imus_.find(imu.name);
  if (it == imus_.end()) {
    return false;
  }

  sensor_msgs::msg::Imu message;
  message.header.stamp = sim_time_;
  message.header.frame_id = imu.frame_id;
  message.orientation.x = imu.state.orientation[0];
  message.orientation.y = imu.state.orientation[1];
  message.orientation.z = imu.state.orientation[2];
  message.orientation.w = imu.state.orientation[3];
  message.orientation_covariance = imu.state.orientation_covariance;
  message.angular_velocity.x = imu.state.angular_velocity[0];
  message.angular_velocity.y = imu.state.angular_velocity[1];
  message.angular_velocity.z = imu.state.angular_velocity[2];
  message.angular_velocity_covariance = imu.state.angular_velocity_covariance;
  message.linear_acceleration.x = imu.state.linear_acceleration[0];
  message.linear_acceleration.y = imu.state.linear_acceleration[1];
  message.linear_acceleration.z = imu.state.linear_acceleration[2];
  message.linear_acceleration_covariance = imu.state.linear_acceleration_covariance;
  it->second.imu->publish(message);
  return true;
}

bool SensorBridge::publish_camera(const CameraData& camera) {
  const auto it = cameras_.find(camera.name);
  if (it == cameras_.end()) {
    return false;
  }

  const auto& binding = it->second;
  if (binding.rgb != nullptr) {
    sensor_msgs::msg::Image message;
    message.header.stamp = sim_time_;
    message.header.frame_id = camera.frame_id;
    message.height = camera.state.image.height;
    message.width = camera.state.image.width;
    message.encoding = camera.state.image.encoding;
    message.is_bigendian = camera.state.image.is_bigendian;
    message.step = camera.state.image.step;
    message.data = camera.state.image.data;
    binding.rgb->publish(message);
  }

  if (binding.depth != nullptr) {
    sensor_msgs::msg::Image message;
    message.header.stamp = sim_time_;
    message.header.frame_id = camera.frame_id;
    message.height = camera.state.depth_image.height;
    message.width = camera.state.depth_image.width;
    message.encoding = camera.state.depth_image.encoding;
    message.is_bigendian = camera.state.depth_image.is_bigendian;
    message.step = camera.state.depth_image.step;
    message.data = camera.state.depth_image.data;
    binding.depth->publish(message);
  }

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = sim_time_;
  info.header.frame_id = camera.frame_id;
  info.width = static_cast<uint32_t>(camera.info.width);
  info.height = static_cast<uint32_t>(camera.info.height);
  info.distortion_model = camera.intrinsics.distortion_model;
  info.d = camera.intrinsics.distortion_coefficients;
  info.k = camera.intrinsics.intrinsic_matrix;
  info.r = camera.intrinsics.rectification_matrix;
  info.p = camera.intrinsics.projection_matrix;
  binding.camera_info->publish(info);
  return true;
}

bool SensorBridge::publish_lidar(const LidarData& lidar) {
  auto it = lidars_.find(lidar.name);
  if (it == lidars_.end()) {
    return false;
  }

  auto& binding = it->second;
  sensor_msgs::msg::LaserScan message;
  message.header.stamp = sim_time_;
  message.header.frame_id = lidar.frame_id;
  message.angle_min = static_cast<float>(lidar.state.laser_scan.angle_min);
  message.angle_max = static_cast<float>(lidar.state.laser_scan.angle_max);
  message.angle_increment = static_cast<float>(lidar.state.laser_scan.angle_increment);
  message.scan_time = static_cast<float>(lidar.state.laser_scan.scan_time);
  message.time_increment = static_cast<float>(lidar.state.laser_scan.time_increment);
  message.range_min = static_cast<float>(lidar.state.laser_scan.range_min);
  message.range_max = static_cast<float>(lidar.state.laser_scan.range_max);
  message.ranges.assign(lidar.state.laser_scan.ranges.begin(), lidar.state.laser_scan.ranges.end());
  if (lidar.state.laser_scan.intensities.empty()) {
    message.intensities.assign(message.ranges.size(), 0.0F);
  } else {
    message.intensities.assign(lidar.state.laser_scan.intensities.begin(),
                               lidar.state.laser_scan.intensities.end());
  }
  binding.scan->publish(message);
  return true;
}

}  // namespace franka_hardware
