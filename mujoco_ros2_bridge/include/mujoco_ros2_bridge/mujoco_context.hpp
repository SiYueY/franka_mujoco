#pragma once

#include <hardware_interface/hardware_info.hpp>
#include <memory>
#include <optional>

#include "mujoco_simulation/mujoco_simulation.hpp"

namespace mujoco_ros2_bridge {

class MujocoContext {
 public:
  static void set_simulation(
      const std::shared_ptr<mujoco_simulation::MuJoCoSimulation>& simulation);
  static std::shared_ptr<mujoco_simulation::MuJoCoSimulation> simulation();
  static void set_hardware_info(const hardware_interface::HardwareInfo& hardware_info);
  static std::optional<hardware_interface::HardwareInfo> hardware_info();
  static void clear_for_testing();

 private:
  MujocoContext() = delete;
};

}  // namespace mujoco_ros2_bridge
