#include "mujoco_ros2_bridge/mujoco_context.hpp"

#include <mutex>

namespace mujoco_ros2_bridge {
namespace {
std::mutex& context_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::shared_ptr<mujoco_simulation::MuJoCoSimulation>& simulation_instance() {
  static std::shared_ptr<mujoco_simulation::MuJoCoSimulation> simulation;
  return simulation;
}

std::optional<hardware_interface::HardwareInfo>& stored_hardware_info() {
  static std::optional<hardware_interface::HardwareInfo> hardware_info;
  return hardware_info;
}
}  // namespace

void MujocoContext::set_simulation(
    const std::shared_ptr<mujoco_simulation::MuJoCoSimulation>& simulation) {
  std::lock_guard<std::mutex> lock(context_mutex());
  simulation_instance() = simulation;
}

std::shared_ptr<mujoco_simulation::MuJoCoSimulation> MujocoContext::simulation() {
  std::lock_guard<std::mutex> lock(context_mutex());
  return simulation_instance();
}

void MujocoContext::set_hardware_info(const hardware_interface::HardwareInfo& hardware_info) {
  std::lock_guard<std::mutex> lock(context_mutex());
  stored_hardware_info() = hardware_info;
}

std::optional<hardware_interface::HardwareInfo> MujocoContext::hardware_info() {
  std::lock_guard<std::mutex> lock(context_mutex());
  return stored_hardware_info();
}

void MujocoContext::clear_for_testing() {
  std::lock_guard<std::mutex> lock(context_mutex());
  simulation_instance().reset();
  stored_hardware_info().reset();
}

}  // namespace mujoco_ros2_bridge
