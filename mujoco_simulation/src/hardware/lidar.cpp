#include "mujoco_simulation/hardware/lidar.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace mujoco_simulation {
namespace {

int parse_beam_index(const std::string& sensor_name, const std::string& prefix) {
  const std::string expected_prefix = prefix + "-";
  if (sensor_name.rfind(expected_prefix, 0) != 0) {
    return -1;
  }

  const std::string suffix = sensor_name.substr(expected_prefix.size());
  if (suffix.empty() || !std::all_of(suffix.begin(), suffix.end(),
                                     [](unsigned char c) { return std::isdigit(c) != 0; })) {
    return -1;
  }
  return std::stoi(suffix);
}

}  // namespace

Lidar::Lidar(const mjModel* model, mjData* data) : model_(model), mj_data_(data) {}

bool Lidar::init(const LidarData& data) {
  data_ = data;
  state_ = {};
  sensor_addresses_.clear();
  last_error_.clear();

  if (model_ == nullptr || mj_data_ == nullptr) {
    return set_error("MuJoCo model/data is not available for lidar '" + data.name + "'.");
  }
  if (data.sensor_prefix.empty()) {
    return set_error("Lidar '" + data.name + "' requires a non-empty sensor_prefix.");
  }
  if (data.angle_increment <= 0.0 || data.angle_max < data.angle_min) {
    return set_error("Lidar '" + data.name + "' has invalid angular configuration.");
  }

  const double span = (data.angle_max - data.angle_min) / data.angle_increment;
  const int beam_count = static_cast<int>(std::llround(span)) + 1;
  if (beam_count <= 0) {
    return set_error("Lidar '" + data.name + "' computed an invalid beam count.");
  }

  sensor_addresses_.assign(static_cast<std::size_t>(beam_count), -1);
  for (int sensor_id = 0; sensor_id < model_->nsensor; ++sensor_id) {
    if (model_->sensor_type[sensor_id] != mjSENS_RANGEFINDER) {
      continue;
    }
    const char* sensor_name = mj_id2name(model_, mjOBJ_SENSOR, sensor_id);
    if (sensor_name == nullptr) {
      continue;
    }

    const int beam_index = parse_beam_index(sensor_name, data.sensor_prefix);
    if (beam_index < 0 || beam_index >= beam_count) {
      continue;
    }
    sensor_addresses_[static_cast<std::size_t>(beam_index)] = model_->sensor_adr[sensor_id];
  }

  if (std::find(sensor_addresses_.begin(), sensor_addresses_.end(), -1) !=
      sensor_addresses_.end()) {
    return set_error("Lidar '" + data.name + "' is missing one or more rangefinder beams.");
  }

  state_.frame_id = data.frame_name;
  state_.angle_min = data.angle_min;
  state_.angle_max = data.angle_max;
  state_.angle_increment = data.angle_increment;
  state_.range_min = data.range_min;
  state_.range_max = data.range_max;
  state_.ranges.assign(sensor_addresses_.size(), -1.0);
  return true;
}

bool Lidar::reset() {
  last_error_.clear();
  std::fill(state_.ranges.begin(), state_.ranges.end(), -1.0);
  return true;
}

bool Lidar::write(const LidarCommand&) {
  last_error_.clear();
  return true;
}

bool Lidar::read(LidarState& state) {
  last_error_.clear();
  if (mj_data_ == nullptr || sensor_addresses_.empty()) {
    return set_error("Lidar '" + data_.name + "' is not initialized.");
  }

  for (std::size_t i = 0; i < sensor_addresses_.size(); ++i) {
    const int address = sensor_addresses_[i];
    const double range = mj_data_->sensordata[address];
    state_.ranges[i] = (range < data_.range_min || range > data_.range_max) ? -1.0 : range;
  }

  state = state_;
  return true;
}

bool Lidar::set_error(const std::string& message) {
  last_error_ = message;
  return false;
}

}  // namespace mujoco_simulation
