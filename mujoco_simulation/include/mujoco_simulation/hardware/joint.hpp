#pragma once

#include <mujoco/mujoco.h>

#include <functional>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <string>
#include <vector>

namespace mujoco_simulation {

enum class MobileBaseType {
  None,
  DifferentialDrive,
  Ackermann,
  Tricycle,
  Mecanum,
  Omni,
  CustomJointGroup,
};

enum class JointRole {
  Manipulator,
  MobileTraction,
  MobileSteering,
  Passive,
};

struct MobileBaseData {
  MobileBaseType type = MobileBaseType::None;
  std::string base_frame_id = "base_link";
  std::string odom_frame_id = "odom";
  std::string feedback_mode = "position";
  std::vector<std::string> traction_joint_names;
  std::vector<std::string> steering_joint_names;
  std::vector<std::string> passive_joint_names;
};

struct JointData {
  std::string name;
  int joint_id = -1;
  int actuator_id = -1;
  int qpos_address = -1;
  int dof_address = -1;
  JointRole role = JointRole::Manipulator;

  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;

  double position_command = 0.0;
  double velocity_command = 0.0;
  double effort_command = 0.0;

  bool has_position_state = false;
  bool has_velocity_state = false;
  bool has_effort_state = false;
  bool has_position_command = false;
  bool has_velocity_command = false;
  bool has_effort_command = false;
};

class MujocoJoints {
 public:
  using ParameterLookup = std::function<std::string(const std::string &, const std::string &)>;

  bool configure(const hardware_interface::HardwareInfo &hardware_info, const mjModel &model,
                 const ParameterLookup &parameter_lookup, std::string *error_message);

  std::vector<hardware_interface::StateInterface> export_state_interfaces(
      const std::vector<hardware_interface::ComponentInfo> &joint_infos);

  std::vector<hardware_interface::CommandInterface> export_command_interfaces(
      const std::vector<hardware_interface::ComponentInfo> &joint_infos);

  void read(const mjData &data);
  void write(mjData &data) const;

  const MobileBaseData &mobile_base() const { return mobile_base_; }
  const std::vector<JointData> &bindings() const { return joints_; }

 private:
  bool configure_mobile_base(const hardware_interface::HardwareInfo &hardware_info,
                             const ParameterLookup &parameter_lookup, std::string *error_message);
  bool bind_joints(const hardware_interface::HardwareInfo &hardware_info, const mjModel &model,
                   const ParameterLookup &parameter_lookup, std::string *error_message);
  int find_actuator_for_joint(const mjModel &model, const std::string &joint_name, int joint_id,
                              const ParameterLookup &parameter_lookup) const;
  JointRole role_for_joint(const std::string &joint_name) const;

  MobileBaseData mobile_base_;
  std::vector<JointData> joints_;
};

}  // namespace mujoco_simulation
