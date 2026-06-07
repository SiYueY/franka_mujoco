#include "mujoco_ros2_bridge/mujoco_system_interface.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_bridge
{

hardware_interface::CallbackReturn MujocoSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  node_ = rclcpp::Node::make_shared("mujoco_ros2_bridge_hardware");

  SimulationConfig config;
  config.model_path = hardware_parameter("mujoco_model_path");
  config.render_mode = parse_render_mode(hardware_parameter("render_mode", "headless"));
  config.publish_clock = hardware_parameter("publish_clock", "true") != "false";
  config.initial_keyframe = hardware_parameter("initial_keyframe");
  const std::string speed = hardware_parameter("sim_speed_factor", "1.0");
  config.sim_speed_factor = std::stod(speed);

  simulation_ = std::make_shared<MuJoCoSimulation>(node_);
  std::string error_message;
  if (!simulation_->initialize(config, &error_message)) {
    RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!bind_joints(&error_message)) {
    RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : joints_) {
    if (joint.has_position_command) {
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position_command);
    }
    if (joint.has_velocity_command) {
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_command);
    }
    if (joint.has_effort_command) {
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort_command);
    }
  }
  return interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  simulation_->start();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  simulation_->stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  simulation_->with_locked_data([this](const mjModel &, const mjData & data) {
    for (auto & joint : joints_) {
      if (joint.qpos_address >= 0) {
        joint.position = data.qpos[joint.qpos_address];
      }
      if (joint.dof_address >= 0) {
        joint.velocity = data.qvel[joint.dof_address];
        joint.effort = data.qfrc_actuator[joint.dof_address] + data.qfrc_applied[joint.dof_address];
      }
    }
  });

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  simulation_->with_locked_data([this](const mjModel &, mjData & data) {
    for (const auto & joint : joints_) {
      if (joint.actuator_id >= 0) {
        if (joint.has_position_command) {
          data.ctrl[joint.actuator_id] = joint.position_command;
        } else if (joint.has_velocity_command) {
          data.ctrl[joint.actuator_id] = joint.velocity_command;
        } else if (joint.has_effort_command) {
          data.ctrl[joint.actuator_id] = joint.effort_command;
        }
      } else if (joint.has_effort_command && joint.dof_address >= 0) {
        data.qfrc_applied[joint.dof_address] = joint.effort_command;
      }
    }
  });

  return hardware_interface::return_type::OK;
}

bool MujocoSystemInterface::bind_joints(std::string * error_message)
{
  const mjModel * model = simulation_->model();
  if (model == nullptr) {
    *error_message = "MuJoCo model is not loaded.";
    return false;
  }

  for (const auto & info_joint : info_.joints) {
    JointBinding binding;
    binding.name = info_joint.name;
    binding.joint_id = mj_name2id(model, mjOBJ_JOINT, binding.name.c_str());
    if (binding.joint_id < 0) {
      *error_message = "MuJoCo joint not found: " + binding.name;
      return false;
    }

    binding.qpos_address = model->jnt_qposadr[binding.joint_id];
    binding.dof_address = model->jnt_dofadr[binding.joint_id];

    for (const auto & command_interface : info_joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        binding.has_position_command = true;
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        binding.has_velocity_command = true;
      } else if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        binding.has_effort_command = true;
      }
    }

    binding.actuator_id = find_actuator_for_joint(binding.name, binding.joint_id);
    if ((binding.has_position_command || binding.has_velocity_command) && binding.actuator_id < 0) {
      *error_message = "MuJoCo actuator not found for joint: " + binding.name;
      return false;
    }

    joints_.push_back(binding);
  }

  return true;
}

int MujocoSystemInterface::find_actuator_for_joint(const std::string & joint_name, int joint_id) const
{
  const mjModel * model = simulation_->model();
  auto actuator_param = std::string("mujoco_actuator_name.");
  actuator_param += joint_name;
  const std::string explicit_name = hardware_parameter(actuator_param);

  if (!explicit_name.empty()) {
    return mj_name2id(model, mjOBJ_ACTUATOR, explicit_name.c_str());
  }

  for (int i = 0; i < model->nu; ++i) {
    if (model->actuator_trntype[i] == mjTRN_JOINT && model->actuator_trnid[2 * i] == joint_id) {
      return i;
    }
  }

  return mj_name2id(model, mjOBJ_ACTUATOR, joint_name.c_str());
}

std::string MujocoSystemInterface::hardware_parameter(
  const std::string & key, const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

}  // namespace mujoco_ros2_bridge

PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_bridge::MujocoSystemInterface,
  hardware_interface::SystemInterface)
