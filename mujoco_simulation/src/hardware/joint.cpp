#include <algorithm>
#include <cctype>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <set>
#include <sstream>
#include <stdexcept>

#include "mujoco_simulation/mujoco_joint.hpp"

namespace mujoco_simulation {
namespace {

std::string trim(std::string value) {
  value.erase(value.begin(), std::find_if(value.begin(), value.end(),
                                          [](unsigned char c) { return std::isspace(c) == 0; }));
  value.erase(std::find_if(value.rbegin(), value.rend(),
                           [](unsigned char c) { return std::isspace(c) == 0; })
                  .base(),
              value.end());
  if (value.size() >= 2 && ((value.front() == '[' && value.back() == ']') ||
                            (value.front() == '"' && value.back() == '"'))) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

std::vector<std::string> split_list(const std::string &value) {
  std::vector<std::string> items;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = trim(item);
    if (!item.empty()) {
      items.push_back(item);
    }
  }
  return items;
}

MobileBaseType parse_mobile_base_type(const std::string &value) {
  if (value.empty() || value == "none") {
    return MobileBaseType::None;
  }
  if (value == "differential_drive") {
    return MobileBaseType::DifferentialDrive;
  }
  if (value == "ackermann") {
    return MobileBaseType::Ackermann;
  }
  if (value == "tricycle") {
    return MobileBaseType::Tricycle;
  }
  if (value == "mecanum") {
    return MobileBaseType::Mecanum;
  }
  if (value == "omni") {
    return MobileBaseType::Omni;
  }
  if (value == "custom_joint_group") {
    return MobileBaseType::CustomJointGroup;
  }
  throw std::invalid_argument("Unsupported mobile_base.type: " + value);
}

}  // namespace

bool MujocoJoints::configure(const hardware_interface::HardwareInfo &hardware_info,
                             const mjModel &model, const ParameterLookup &parameter_lookup,
                             std::string *error_message) {
  if (!configure_mobile_base(hardware_info, parameter_lookup, error_message)) {
    return false;
  }
  return bind_joints(hardware_info, model, parameter_lookup, error_message);
}

std::vector<hardware_interface::StateInterface> MujocoJoints::export_state_interfaces(
    const std::vector<hardware_interface::ComponentInfo> &joint_infos) {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto &joint : joints_) {
    const auto joint_it =
        std::find_if(joint_infos.begin(), joint_infos.end(),
                     [&joint](const auto &info_joint) { return info_joint.name == joint.name; });
    if (joint_it == joint_infos.end()) {
      continue;
    }
    for (const auto &state_interface : joint_it->state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(joint.name, state_interface.name, &joint.position);
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(joint.name, state_interface.name, &joint.velocity);
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        interfaces.emplace_back(joint.name, state_interface.name, &joint.effort);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoJoints::export_command_interfaces(
    const std::vector<hardware_interface::ComponentInfo> &joint_infos) {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto &joint : joints_) {
    const auto joint_it =
        std::find_if(joint_infos.begin(), joint_infos.end(),
                     [&joint](const auto &info_joint) { return info_joint.name == joint.name; });
    if (joint_it == joint_infos.end()) {
      continue;
    }

    for (const auto &command_interface : joint_it->command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(joint.name, command_interface.name, &joint.position_command);
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(joint.name, command_interface.name, &joint.velocity_command);
      } else if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        interfaces.emplace_back(joint.name, command_interface.name, &joint.effort_command);
      }
    }
  }
  return interfaces;
}

void MujocoJoints::read(const mjData &data) {
  for (auto &joint : joints_) {
    if (joint.qpos_address >= 0) {
      joint.position = data.qpos[joint.qpos_address];
    }
    if (joint.dof_address >= 0) {
      joint.velocity = data.qvel[joint.dof_address];
      joint.effort = data.qfrc_actuator[joint.dof_address] + data.qfrc_applied[joint.dof_address];
    }
  }
}

void MujocoJoints::write(mjData &data) const {
  for (const auto &joint : joints_) {
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
}

bool MujocoJoints::configure_mobile_base(const hardware_interface::HardwareInfo &,
                                         const ParameterLookup &parameter_lookup,
                                         std::string *error_message) {
  mobile_base_ = {};
  mobile_base_.base_frame_id = parameter_lookup("mobile_base.base_frame_id", "base_link");
  mobile_base_.odom_frame_id = parameter_lookup("mobile_base.odom_frame_id", "odom");
  mobile_base_.feedback_mode = parameter_lookup("mobile_base.feedback_mode", "position");
  mobile_base_.traction_joint_names =
      split_list(parameter_lookup("mobile_base.traction_joints", ""));
  mobile_base_.steering_joint_names =
      split_list(parameter_lookup("mobile_base.steering_joints", ""));
  mobile_base_.passive_joint_names = split_list(parameter_lookup("mobile_base.passive_joints", ""));
  mobile_base_.type = parse_mobile_base_type(parameter_lookup("mobile_base.type", "none"));

  if (mobile_base_.type == MobileBaseType::None) {
    return true;
  }
  if (mobile_base_.type != MobileBaseType::DifferentialDrive &&
      mobile_base_.traction_joint_names.empty()) {
    if (error_message != nullptr) {
      *error_message =
          "mobile_base.traction_joints must not be empty when mobile_base.type is configured.";
    }
    return false;
  }
  if ((mobile_base_.type == MobileBaseType::Ackermann ||
       mobile_base_.type == MobileBaseType::Tricycle ||
       mobile_base_.type == MobileBaseType::CustomJointGroup) &&
      mobile_base_.steering_joint_names.empty()) {
    if (error_message != nullptr) {
      *error_message = "mobile_base.steering_joints must not be empty for steering mobile bases.";
    }
    return false;
  }
  return true;
}

bool MujocoJoints::bind_joints(const hardware_interface::HardwareInfo &hardware_info,
                               const mjModel &model, const ParameterLookup &parameter_lookup,
                               std::string *error_message) {
  joints_.clear();
  for (const auto &info_joint : hardware_info.joints) {
    JointData binding;
    binding.name = info_joint.name;
    binding.role = role_for_joint(binding.name);
    binding.joint_id = mj_name2id(&model, mjOBJ_JOINT, binding.name.c_str());
    if (binding.joint_id < 0) {
      *error_message = "MuJoCo joint not found: " + binding.name;
      return false;
    }

    binding.qpos_address = model.jnt_qposadr[binding.joint_id];
    binding.dof_address = model.jnt_dofadr[binding.joint_id];

    for (const auto &state_interface : info_joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        binding.has_position_state = true;
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        binding.has_velocity_state = true;
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        binding.has_effort_state = true;
      }
    }

    for (const auto &command_interface : info_joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        binding.has_position_command = true;
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        binding.has_velocity_command = true;
      } else if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
        binding.has_effort_command = true;
      }
    }

    binding.actuator_id =
        find_actuator_for_joint(model, binding.name, binding.joint_id, parameter_lookup);
    if ((binding.has_position_command || binding.has_velocity_command ||
         binding.has_effort_command) &&
        binding.actuator_id < 0 && !binding.has_effort_command) {
      *error_message = "MuJoCo actuator not found for joint: " + binding.name;
      return false;
    }

    joints_.push_back(binding);
  }

  if (mobile_base_.type != MobileBaseType::None) {
    std::set<std::string> available_names;
    for (const auto &joint : joints_) {
      available_names.insert(joint.name);
    }
    for (const auto &name : mobile_base_.traction_joint_names) {
      if (available_names.count(name) == 0U) {
        *error_message =
            "Configured mobile base traction joint not found in ros2_control joints: " + name;
        return false;
      }
    }
    for (const auto &name : mobile_base_.steering_joint_names) {
      if (available_names.count(name) == 0U) {
        *error_message =
            "Configured mobile base steering joint not found in ros2_control joints: " + name;
        return false;
      }
    }
  }

  return true;
}

int MujocoJoints::find_actuator_for_joint(const mjModel &model, const std::string &joint_name,
                                          int joint_id,
                                          const ParameterLookup &parameter_lookup) const {
  auto actuator_param = std::string("mujoco_actuator_name.");
  actuator_param += joint_name;
  const std::string explicit_name = parameter_lookup(actuator_param, "");

  if (!explicit_name.empty()) {
    return mj_name2id(&model, mjOBJ_ACTUATOR, explicit_name.c_str());
  }

  for (int i = 0; i < model.nu; ++i) {
    if (model.actuator_trntype[i] == mjTRN_JOINT && model.actuator_trnid[2 * i] == joint_id) {
      return i;
    }
  }

  return mj_name2id(&model, mjOBJ_ACTUATOR, joint_name.c_str());
}

JointRole MujocoJoints::role_for_joint(const std::string &joint_name) const {
  if (std::find(mobile_base_.traction_joint_names.begin(), mobile_base_.traction_joint_names.end(),
                joint_name) != mobile_base_.traction_joint_names.end()) {
    return JointRole::MobileTraction;
  }
  if (std::find(mobile_base_.steering_joint_names.begin(), mobile_base_.steering_joint_names.end(),
                joint_name) != mobile_base_.steering_joint_names.end()) {
    return JointRole::MobileSteering;
  }
  if (std::find(mobile_base_.passive_joint_names.begin(), mobile_base_.passive_joint_names.end(),
                joint_name) != mobile_base_.passive_joint_names.end()) {
    return JointRole::Passive;
  }
  return JointRole::Manipulator;
}

}  // namespace mujoco_simulation
