#include "mujoco_simulation/hardware/joint.hpp"

namespace mujoco_simulation {

Joint::Joint(const mjModel* model, mjData* data) : model_(model), mj_data_(data) {}

bool Joint::init(const JointInfo& data) {
  data_ = data;
  command_ = {};
  state_ = {};
  last_error_.clear();
  joint_id_ = -1;
  qpos_address_ = -1;
  dof_address_ = -1;
  actuator_id_ = -1;
  joint_type_ = JointType::Unknown;
  actuator_type_ = ActuatorType::Unknown;
  command_mode_ = CommandInterfaceType::None;

  if (model_ == nullptr || mj_data_ == nullptr) {
    return set_error("MuJoCo model/data is not available for joint '" + data.name + "'.");
  }

  joint_id_ = mj_name2id(model_, mjOBJ_JOINT, data.name.c_str());
  if (joint_id_ < 0) {
    return set_error("MuJoCo joint not found: " + data.name);
  }

  joint_type_ = parse_joint_type(model_->jnt_type[joint_id_]);
  if (joint_type_ != JointType::Hinge && joint_type_ != JointType::Slide) {
    return set_error("Joint '" + data.name +
                     "' uses an unsupported MuJoCo joint type; current Joint abstraction only "
                     "supports 1-DoF joints (hinge/slide).");
  }

  qpos_address_ = model_->jnt_qposadr[joint_id_];
  dof_address_ = model_->jnt_dofadr[joint_id_];
  if (qpos_address_ < 0 || dof_address_ < 0) {
    return set_error("MuJoCo joint has invalid qpos/dof addresses: " + data.name);
  }

  command_mode_ = data.command_mode;

  actuator_id_ = find_actuator_id();
  actuator_type_ = parse_actuator_type(actuator_id_);
  if (!validate_command_mode()) {
    return false;
  }

  return reset();
}

bool Joint::reset() {
  last_error_.clear();
  command_ = {};
  return read(state_);
}

bool Joint::write(const JointCommand& command) {
  last_error_.clear();
  if (mj_data_ == nullptr || joint_id_ < 0) {
    return set_error("Joint '" + data_.name + "' is not initialized.");
  }

  command_ = command;
  if (command_mode_ == CommandInterfaceType::Position) {
    if (actuator_type_ == ActuatorType::Passive || actuator_id_ < 0) {
      return set_error("Position command requires an actuator for joint '" + data_.name + "'.");
    }
    mj_data_->ctrl[actuator_id_] = command.position;
  } else if (command_mode_ == CommandInterfaceType::Velocity) {
    if (actuator_type_ == ActuatorType::Passive || actuator_id_ < 0) {
      return set_error("Velocity command requires an actuator for joint '" + data_.name + "'.");
    }
    mj_data_->ctrl[actuator_id_] = command.velocity;
  } else if (command_mode_ == CommandInterfaceType::Effort) {
    if (actuator_type_ == ActuatorType::Motor || actuator_type_ == ActuatorType::Custom) {
      mj_data_->ctrl[actuator_id_] = command.effort;
    } else if (actuator_type_ == ActuatorType::Passive && dof_address_ >= 0) {
      mj_data_->qfrc_applied[dof_address_] = command.effort;
    } else {
      return set_error("Effort command is not supported by the actuator type for joint '" +
                       data_.name + "'.");
    }
  }
  return true;
}

bool Joint::read(JointState& state) {
  last_error_.clear();
  if (mj_data_ == nullptr || qpos_address_ < 0 || dof_address_ < 0) {
    return set_error("Joint '" + data_.name + "' is not initialized.");
  }

  state_.position = mj_data_->qpos[qpos_address_];
  state_.velocity = mj_data_->qvel[dof_address_];
  state_.effort = mj_data_->qfrc_actuator[dof_address_] + mj_data_->qfrc_applied[dof_address_];
  state = state_;
  return true;
}

bool Joint::set_error(const std::string& message) {
  last_error_ = message;
  return false;
}

int Joint::find_actuator_id() const {
  if (model_ == nullptr) {
    return -1;
  }

  if (!data_.actuator_name.empty()) {
    return mj_name2id(model_, mjOBJ_ACTUATOR, data_.actuator_name.c_str());
  }

  for (int actuator_id = 0; actuator_id < model_->nu; ++actuator_id) {
    if (model_->actuator_trntype[actuator_id] != mjTRN_JOINT) {
      continue;
    }
    if (model_->actuator_trnid[2 * actuator_id] == joint_id_) {
      return actuator_id;
    }
  }
  return -1;
}

bool Joint::validate_command_mode() const {
  auto* self = const_cast<Joint*>(this);
  if (command_mode_ == CommandInterfaceType::None) {
    return true;
  }

  if (command_mode_ == CommandInterfaceType::Position) {
    if (actuator_type_ == ActuatorType::Passive) {
      return self->set_error("Position command is not supported for passive joint '" + data_.name +
                             "'.");
    }
    return true;
  }

  if (command_mode_ == CommandInterfaceType::Velocity) {
    if (actuator_type_ == ActuatorType::Passive) {
      return self->set_error("Velocity command is not supported for passive joint '" + data_.name +
                             "'.");
    }
    if (actuator_type_ == ActuatorType::Position) {
      return self->set_error("Velocity command is not supported for position actuator on joint '" +
                             data_.name + "'.");
    }
    return true;
  }

  if (command_mode_ == CommandInterfaceType::Effort) {
    if (actuator_type_ == ActuatorType::Position || actuator_type_ == ActuatorType::Velocity) {
      return self->set_error(
          "Effort command is not supported for position/velocity actuator on joint '" + data_.name +
          "'.");
    }
    return true;
  }

  return true;
}

JointType Joint::parse_joint_type(int mujoco_joint_type) const {
  if (mujoco_joint_type == mjJNT_HINGE) {
    return JointType::Hinge;
  }
  if (mujoco_joint_type == mjJNT_SLIDE) {
    return JointType::Slide;
  }
  if (mujoco_joint_type == mjJNT_BALL) {
    return JointType::Ball;
  }
  if (mujoco_joint_type == mjJNT_FREE) {
    return JointType::Free;
  }
  return JointType::Unknown;
}

ActuatorType Joint::parse_actuator_type(int actuator_id) const {
  if (model_ == nullptr) {
    return ActuatorType::Unknown;
  }
  if (actuator_id < 0) {
    return ActuatorType::Passive;
  }

  const int bias_type = model_->actuator_biastype[actuator_id];
  constexpr int kBiasParamCount = 10;
  const mjtNum* biasprm = model_->actuator_biasprm + actuator_id * kBiasParamCount;

  if (bias_type == mjBIAS_NONE) {
    return ActuatorType::Motor;
  }
  if (bias_type == mjBIAS_AFFINE && biasprm[1] != 0) {
    return ActuatorType::Position;
  }
  if (bias_type == mjBIAS_AFFINE && biasprm[1] == 0 && biasprm[2] != 0) {
    return ActuatorType::Velocity;
  }
  return ActuatorType::Custom;
}

}  // namespace mujoco_simulation
