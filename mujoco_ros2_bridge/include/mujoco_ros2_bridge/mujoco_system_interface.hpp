#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "mujoco_ros2_bridge/mujoco_simulation.hpp"

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
  struct JointBinding
  {
    std::string name;
    int joint_id = -1;
    int actuator_id = -1;
    int qpos_address = -1;
    int dof_address = -1;

    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;

    double position_command = 0.0;
    double velocity_command = 0.0;
    double effort_command = 0.0;

    bool has_position_command = false;
    bool has_velocity_command = false;
    bool has_effort_command = false;
  };

  bool bind_joints(std::string * error_message);
  int find_actuator_for_joint(const std::string & joint_name, int joint_id) const;
  std::string hardware_parameter(const std::string & key, const std::string & default_value = "") const;

  hardware_interface::HardwareInfo info_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MuJoCoSimulation> simulation_;
  std::vector<JointBinding> joints_;
};

}  // namespace mujoco_ros2_bridge
