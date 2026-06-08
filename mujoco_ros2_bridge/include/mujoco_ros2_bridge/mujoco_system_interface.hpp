#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

#include "mujoco_simulation/mujoco_imu.hpp"
#include "mujoco_simulation/mujoco_joint.hpp"
#include "mujoco_simulation/mujoco_simulation.hpp"

namespace mujoco_ros2_bridge
{

class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

 private:
  std::string hardware_parameter(const std::string & key, const std::string & default_value = "") const;

  std::shared_ptr<mujoco_simulation::MuJoCoSimulation> simulation_;
  std::unique_ptr<mujoco_simulation::MujocoJoints> joints_;
  std::unique_ptr<mujoco_simulation::MujocoImus> imu_;
};

}  // namespace mujoco_ros2_bridge
