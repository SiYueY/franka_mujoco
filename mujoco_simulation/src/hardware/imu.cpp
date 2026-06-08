#include <algorithm>

#include "mujoco_simulation/mujoco_imu.hpp"

namespace mujoco_simulation {
namespace {

std::string sensor_parameter(const hardware_interface::ComponentInfo& sensor,
                             const std::string& key, const std::string& default_value = "") {
  const auto it = sensor.parameters.find(key);
  return it == sensor.parameters.end() ? default_value : it->second;
}

}  // namespace

bool MujocoImus::configure(const std::vector<hardware_interface::ComponentInfo>& sensors,
                           const mjModel& model, std::string* error_message) {
  imu_bindings_.clear();
  for (const auto& sensor : sensors) {
    if (sensor_parameter(sensor, "mujoco_type") != "imu") {
      continue;
    }

    ImuData binding;
    binding.name = sensor.name;
    binding.orientation_sensor_name = sensor_parameter(sensor, "mujoco_orientation_sensor");
    binding.gyro_sensor_name = sensor_parameter(sensor, "mujoco_gyro_sensor");
    binding.accel_sensor_name = sensor_parameter(sensor, "mujoco_accel_sensor");
    if (binding.orientation_sensor_name.empty() || binding.gyro_sensor_name.empty() ||
        binding.accel_sensor_name.empty()) {
      *error_message =
          "IMU sensor '" + sensor.name +
          "' requires mujoco_orientation_sensor, mujoco_gyro_sensor, and mujoco_accel_sensor.";
      return false;
    }

    const int orientation_id =
        mj_name2id(&model, mjOBJ_SENSOR, binding.orientation_sensor_name.c_str());
    const int gyro_id = mj_name2id(&model, mjOBJ_SENSOR, binding.gyro_sensor_name.c_str());
    const int accel_id = mj_name2id(&model, mjOBJ_SENSOR, binding.accel_sensor_name.c_str());
    if (orientation_id < 0 || gyro_id < 0 || accel_id < 0) {
      *error_message = "One or more MuJoCo IMU sensors are missing for '" + sensor.name + "'.";
      return false;
    }

    binding.orientation_address = model.sensor_adr[orientation_id];
    binding.gyro_address = model.sensor_adr[gyro_id];
    binding.accel_address = model.sensor_adr[accel_id];
    imu_bindings_.push_back(binding);
  }
  return true;
}

std::vector<hardware_interface::StateInterface> MujocoImus::export_state_interfaces(
    const std::vector<hardware_interface::ComponentInfo>& sensors) {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto& imu : imu_bindings_) {
    const auto sensor_it = std::find_if(sensors.begin(), sensors.end(), [&imu](const auto& sensor) {
      return sensor.name == imu.name;
    });
    if (sensor_it == sensors.end()) {
      continue;
    }
    for (const auto& state_interface : sensor_it->state_interfaces) {
      if (state_interface.name == "orientation.x") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.orientation[0]);
      } else if (state_interface.name == "orientation.y") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.orientation[1]);
      } else if (state_interface.name == "orientation.z") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.orientation[2]);
      } else if (state_interface.name == "orientation.w") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.orientation[3]);
      } else if (state_interface.name == "angular_velocity.x") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.angular_velocity[0]);
      } else if (state_interface.name == "angular_velocity.y") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.angular_velocity[1]);
      } else if (state_interface.name == "angular_velocity.z") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.angular_velocity[2]);
      } else if (state_interface.name == "linear_acceleration.x") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.linear_acceleration[0]);
      } else if (state_interface.name == "linear_acceleration.y") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.linear_acceleration[1]);
      } else if (state_interface.name == "linear_acceleration.z") {
        interfaces.emplace_back(imu.name, state_interface.name, &imu.linear_acceleration[2]);
      }
    }
  }
  return interfaces;
}

void MujocoImus::read(const mjData& data) {
  for (auto& imu : imu_bindings_) {
    imu.orientation[3] = data.sensordata[imu.orientation_address];
    imu.orientation[0] = data.sensordata[imu.orientation_address + 1];
    imu.orientation[1] = data.sensordata[imu.orientation_address + 2];
    imu.orientation[2] = data.sensordata[imu.orientation_address + 3];
    imu.angular_velocity[0] = data.sensordata[imu.gyro_address];
    imu.angular_velocity[1] = data.sensordata[imu.gyro_address + 1];
    imu.angular_velocity[2] = data.sensordata[imu.gyro_address + 2];
    imu.linear_acceleration[0] = data.sensordata[imu.accel_address];
    imu.linear_acceleration[1] = data.sensordata[imu.accel_address + 1];
    imu.linear_acceleration[2] = data.sensordata[imu.accel_address + 2];
  }
}

}  // namespace mujoco_simulation
