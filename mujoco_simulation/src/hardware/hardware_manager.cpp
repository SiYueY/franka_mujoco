#include "mujoco_simulation/hardware/hardware_manager.hpp"

#include <utility>

namespace mujoco_simulation {
namespace {

template <typename DeviceMap, typename State>
bool read_named_device(const std::string& name, DeviceMap& devices, std::string& last_error,
                       State& state) {
  const auto it = devices.find(name);
  if (it == devices.end()) {
    last_error = "Hardware device not found: " + name;
    return false;
  }
  if (!it->second->read(state)) {
    last_error = it->second->last_error();
    return false;
  }
  last_error.clear();
  return true;
}

template <typename DeviceMap>
bool unregister_named_device(const std::string& name, DeviceMap& devices, std::string& last_error,
                             const char* device_type) {
  const auto it = devices.find(name);
  if (it == devices.end()) {
    last_error = std::string(device_type) + " not found: " + name;
    return false;
  }
  devices.erase(it);
  last_error.clear();
  return true;
}

}  // namespace

HardwareManager::HardwareManager(const mjModel* model, mjData* data, mjvScene* scene,
                                 mjrContext* render_context)
    : model_(model), mj_data_(data), scene_(scene), render_context_(render_context) {}

bool HardwareManager::register_joint(const JointInfo& data) {
  if (data.name.empty()) {
    return set_error("Joint name must not be empty.");
  }
  if (joints_.find(data.name) != joints_.end()) {
    return set_error("Joint already registered: " + data.name);
  }

  auto device = std::make_unique<Joint>(model_, mj_data_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  joints_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::register_imu(const ImuInfo& data) {
  if (data.name.empty()) {
    return set_error("IMU name must not be empty.");
  }
  if (imus_.find(data.name) != imus_.end()) {
    return set_error("IMU already registered: " + data.name);
  }

  auto device = std::make_unique<Imu>(model_, mj_data_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  imus_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::register_camera(const CameraSpec& data) {
  if (data.name.empty()) {
    return set_error("Camera name must not be empty.");
  }
  if (cameras_.find(data.name) != cameras_.end()) {
    return set_error("Camera already registered: " + data.name);
  }

  auto device = std::make_unique<Camera>(model_, mj_data_, scene_, render_context_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  cameras_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::register_lidar(const LidarInfo& data) {
  if (data.name.empty()) {
    return set_error("Lidar name must not be empty.");
  }
  if (lidars_.find(data.name) != lidars_.end()) {
    return set_error("Lidar already registered: " + data.name);
  }

  auto device = std::make_unique<Lidar>(model_, mj_data_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  lidars_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::register_mobile_base(const MobileBaseData& data) {
  if (data.name.empty()) {
    return set_error("Mobile base name must not be empty.");
  }
  if (mobile_bases_.find(data.name) != mobile_bases_.end()) {
    return set_error("Mobile base already registered: " + data.name);
  }

  std::vector<Joint*> traction_joints;
  traction_joints.reserve(data.traction_joint_names.size());
  for (const auto& joint_name : data.traction_joint_names) {
    const auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
      return set_error("Mobile base traction joint not found: " + joint_name);
    }
    traction_joints.push_back(it->second.get());
  }

  auto device = std::make_unique<MobileBase>(traction_joints);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  mobile_bases_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::unregister_joint(const std::string& name) {
  return unregister_named_device(name, joints_, last_error_, "Joint");
}

bool HardwareManager::unregister_imu(const std::string& name) {
  return unregister_named_device(name, imus_, last_error_, "IMU");
}

bool HardwareManager::unregister_camera(const std::string& name) {
  return unregister_named_device(name, cameras_, last_error_, "Camera");
}

bool HardwareManager::unregister_lidar(const std::string& name) {
  return unregister_named_device(name, lidars_, last_error_, "Lidar");
}

bool HardwareManager::unregister_mobile_base(const std::string& name) {
  return unregister_named_device(name, mobile_bases_, last_error_, "Mobile base");
}

bool HardwareManager::reset_all() {
  return reset_device_map(joints_) && reset_device_map(imus_) && reset_device_map(cameras_) &&
         reset_device_map(lidars_) && reset_device_map(mobile_bases_);
}

bool HardwareManager::write_joint(const std::string& name, const JointCommand& command) {
  const auto it = joints_.find(name);
  if (it == joints_.end()) {
    return set_error("Joint not found: " + name);
  }
  if (!it->second->write(command)) {
    return set_error(it->second->last_error());
  }
  last_error_.clear();
  return true;
}

bool HardwareManager::read_joint(const std::string& name, JointState& state) {
  return read_named_device(name, joints_, last_error_, state);
}

bool HardwareManager::read_imu(const std::string& name, ImuState& state) {
  return read_named_device(name, imus_, last_error_, state);
}

bool HardwareManager::read_camera(const std::string& name, CameraState& state) {
  return read_named_device(name, cameras_, last_error_, state);
}

bool HardwareManager::read_lidar(const std::string& name, LidarState& state) {
  return read_named_device(name, lidars_, last_error_, state);
}

bool HardwareManager::write_mobile_base(const std::string& name, const MobileBaseCommand& command) {
  const auto it = mobile_bases_.find(name);
  if (it == mobile_bases_.end()) {
    return set_error("Mobile base not found: " + name);
  }
  if (!it->second->write(command)) {
    return set_error(it->second->last_error());
  }
  last_error_.clear();
  return true;
}

bool HardwareManager::read_mobile_base(const std::string& name, MobileBaseState& state) {
  return read_named_device(name, mobile_bases_, last_error_, state);
}

std::unordered_map<std::string, JointState> HardwareManager::read_joint_states() {
  return read_all_from<std::unordered_map<std::string, std::unique_ptr<Joint>>, JointState>(
      joints_);
}

std::unordered_map<std::string, LidarState> HardwareManager::read_lidar_states() {
  return read_all_from<std::unordered_map<std::string, std::unique_ptr<Lidar>>, LidarState>(
      lidars_);
}

std::unordered_map<std::string, MobileBaseState> HardwareManager::read_mobile_base_states() {
  return read_all_from<std::unordered_map<std::string, std::unique_ptr<MobileBase>>,
                       MobileBaseState>(mobile_bases_);
}

bool HardwareManager::set_error(const std::string& message) {
  last_error_ = message;
  return false;
}

template <typename DeviceMap>
bool HardwareManager::reset_device_map(DeviceMap& devices) {
  for (auto& [name, device] : devices) {
    if (!device->reset()) {
      return set_error(device->last_error().empty() ? ("Failed to reset device: " + name)
                                                    : device->last_error());
    }
  }
  last_error_.clear();
  return true;
}

template <typename DeviceMap, typename State>
std::unordered_map<std::string, State> HardwareManager::read_all_from(DeviceMap& devices) {
  std::unordered_map<std::string, State> result;
  for (auto& [name, device] : devices) {
    State state;
    if (!device->read(state)) {
      set_error(device->last_error().empty() ? ("Failed to read device: " + name)
                                             : device->last_error());
      return {};
    }
    result.emplace(name, std::move(state));
  }
  last_error_.clear();
  return result;
}

template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Joint>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Imu>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Camera>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Lidar>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<MobileBase>>& devices);

template std::unordered_map<std::string, JointState> HardwareManager::read_all_from(
    std::unordered_map<std::string, std::unique_ptr<Joint>>& devices);
template std::unordered_map<std::string, LidarState> HardwareManager::read_all_from(
    std::unordered_map<std::string, std::unique_ptr<Lidar>>& devices);
template std::unordered_map<std::string, MobileBaseState> HardwareManager::read_all_from(
    std::unordered_map<std::string, std::unique_ptr<MobileBase>>& devices);

}  // namespace mujoco_simulation
