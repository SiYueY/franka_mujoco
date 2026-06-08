#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <utility>

#include "mujoco_simulation/mujoco_lidar.hpp"

namespace mujoco_simulation {
namespace {

std::string sensor_parameter(const hardware_interface::ComponentInfo &sensor,
                             const std::string &key, const std::string &default_value = "") {
  const auto it = sensor.parameters.find(key);
  return it == sensor.parameters.end() ? default_value : it->second;
}

std::pair<std::string, int> parse_beam_name(const std::string &sensor_name) {
  const auto split = sensor_name.find_last_of('-');
  if (split == std::string::npos) {
    return {sensor_name, -1};
  }
  const auto prefix = sensor_name.substr(0, split);
  const auto beam_string = sensor_name.substr(split + 1);
  if (beam_string.empty() || !std::all_of(beam_string.begin(), beam_string.end(),
                                          [](unsigned char c) { return std::isdigit(c) != 0; })) {
    return {prefix, -1};
  }
  return {prefix, std::stoi(beam_string)};
}

}  // namespace

MujocoLidars::MujocoLidars(rclcpp::Node::SharedPtr node, MuJoCoSimulation *simulation)
    : node_(std::move(node)), simulation_(simulation) {}

MujocoLidars::~MujocoLidars() { stop(); }

bool MujocoLidars::register_lidars(const hardware_interface::HardwareInfo &hardware_info,
                                   std::string *error_message) {
  lidars_.clear();
  const mjModel *model = simulation_ != nullptr ? simulation_->model() : nullptr;
  if (model == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo model is not loaded for lidar registration.";
    }
    return false;
  }

  for (const auto &sensor : hardware_info.sensors) {
    if (sensor_parameter(sensor, "mujoco_type") != "lidar") {
      continue;
    }

    const std::string prefix = sensor_parameter(sensor, "mujoco_sensor_prefix");
    const std::string frame_name = sensor_parameter(sensor, "frame_name");
    if (prefix.empty() || frame_name.empty()) {
      if (error_message != nullptr) {
        *error_message =
            "Lidar sensor '" + sensor.name + "' requires mujoco_sensor_prefix and frame_name.";
      }
      return false;
    }

    LidarData binding;
    binding.sensor_name = sensor.name;
    binding.sensor_prefix = prefix;
    binding.frame_name = frame_name;
    binding.scan_topic = sensor_parameter(sensor, "scan_topic", "/" + sensor.name + "/scan");
    binding.publish_rate = std::stod(sensor_parameter(sensor, "publish_rate", "5.0"));
    binding.angle_min = std::stod(sensor_parameter(sensor, "angle_min"));
    binding.angle_max = std::stod(sensor_parameter(sensor, "angle_max"));
    binding.angle_increment = std::stod(sensor_parameter(sensor, "angle_increment"));
    binding.range_min = std::stod(sensor_parameter(sensor, "range_min"));
    binding.range_max = std::stod(sensor_parameter(sensor, "range_max"));
    if (binding.angle_increment <= 0.0 || binding.angle_max < binding.angle_min) {
      if (error_message != nullptr) {
        *error_message = "Lidar sensor '" + sensor.name + "' has invalid angular configuration.";
      }
      return false;
    }

    const int beam_count = static_cast<int>(std::llround((binding.angle_max - binding.angle_min) /
                                                         binding.angle_increment)) +
                           1;
    binding.sensor_indices.assign(static_cast<std::size_t>(beam_count), -1);
    for (int idx = 0; idx < model->nsensor; ++idx) {
      if (model->sensor_type[idx] != mjSENS_RANGEFINDER) {
        continue;
      }
      const char *name = mj_id2name(model, mjOBJ_SENSOR, idx);
      if (name == nullptr) {
        continue;
      }
      const auto [beam_prefix, beam_index] = parse_beam_name(name);
      if (beam_prefix != prefix || beam_index < 0 || beam_index >= beam_count) {
        continue;
      }
      binding.sensor_indices[static_cast<std::size_t>(beam_index)] = model->sensor_adr[idx];
    }
    if (std::find(binding.sensor_indices.begin(), binding.sensor_indices.end(), -1) !=
        binding.sensor_indices.end()) {
      if (error_message != nullptr) {
        *error_message =
            "Lidar sensor '" + sensor.name + "' is missing one or more rangefinder beams.";
      }
      return false;
    }

    binding.scan_msg.header.frame_id = binding.frame_name;
    binding.scan_msg.angle_min = static_cast<float>(binding.angle_min);
    binding.scan_msg.angle_max = static_cast<float>(binding.angle_max);
    binding.scan_msg.angle_increment = static_cast<float>(binding.angle_increment);
    binding.scan_msg.range_min = static_cast<float>(binding.range_min);
    binding.scan_msg.range_max = static_cast<float>(binding.range_max);
    binding.scan_msg.scan_time = static_cast<float>(1.0 / binding.publish_rate);
    binding.scan_publisher =
        node_->create_publisher<sensor_msgs::msg::LaserScan>(binding.scan_topic, rclcpp::QoS(1));
    lidars_.push_back(std::move(binding));
  }

  if (!lidars_.empty()) {
    sensor_data_.resize(static_cast<std::size_t>(model->nsensordata));
  }
  return true;
}

void MujocoLidars::start() {
  if (lidars_.empty() || publish_lidar_.exchange(true)) {
    return;
  }
  publish_thread_ = std::thread(&MujocoLidars::update_loop, this);
}

void MujocoLidars::stop() {
  publish_lidar_.store(false);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void MujocoLidars::update_loop() {
  double max_publish_rate = 1.0;
  for (const auto &lidar : lidars_) {
    max_publish_rate = std::max(max_publish_rate, lidar.publish_rate);
  }
  rclcpp::Rate rate(max_publish_rate);
  while (rclcpp::ok() && publish_lidar_.load()) {
    update_once();
    rate.sleep();
  }
}

void MujocoLidars::update_once() {
  if (sensor_data_.empty()) {
    return;
  }

  simulation_->with_locked_data([this](const mjModel &, mjData &data) {
    std::memcpy(sensor_data_.data(), data.sensordata, sensor_data_.size() * sizeof(mjtNum));
  });

  const auto now = node_->now();
  for (auto &lidar : lidars_) {
    lidar.scan_msg.header.stamp = now;
    lidar.scan_msg.ranges.resize(lidar.sensor_indices.size(), -1.0f);
    for (std::size_t i = 0; i < lidar.sensor_indices.size(); ++i) {
      const auto range =
          static_cast<float>(sensor_data_[static_cast<std::size_t>(lidar.sensor_indices[i])]);
      lidar.scan_msg.ranges[i] =
          (range < lidar.scan_msg.range_min || range > lidar.scan_msg.range_max) ? -1.0f : range;
    }
    lidar.scan_publisher->publish(lidar.scan_msg);
  }
}

}  // namespace mujoco_simulation
