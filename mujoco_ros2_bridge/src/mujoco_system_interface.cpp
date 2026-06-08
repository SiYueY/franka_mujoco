#include "mujoco_ros2_bridge/mujoco_system_interface.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "mujoco_ros2_bridge/mujoco_context.hpp"

namespace mujoco_ros2_bridge
{

hardware_interface::CallbackReturn MujocoSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  MujocoContext::set_hardware_info(info_);
  simulation_ = MujocoContext::simulation();
  const auto logger = rclcpp::get_logger("mujoco_ros2_bridge_hardware");
  if (simulation_ == nullptr) {
    RCLCPP_ERROR(logger, "Shared MuJoCoSimulation is not available in MujocoContext.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!simulation_->is_initialized()) {
    mujoco_simulation::SimulationConfig config;
    config.model_path = hardware_parameter("mujoco_model_path");
    config.render_mode =
        mujoco_simulation::parse_render_mode(hardware_parameter("render_mode", "headless"));
    config.initial_keyframe = hardware_parameter("initial_keyframe");
    config.sim_speed_factor = std::stod(hardware_parameter("sim_speed_factor", "1.0"));

    std::string error_message;
    if (!simulation_->initialize(config, &error_message)) {
      RCLCPP_ERROR(logger, "%s", error_message.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const mjModel *model = simulation_->model();
  if (model == nullptr) {
    RCLCPP_ERROR(logger, "MuJoCo model is not loaded.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_ = std::make_unique<mujoco_simulation::MujocoJoints>();
  std::string error_message;
  if (!joints_->configure(
          info_, *model,
          [this](const std::string &key, const std::string &default_value) {
            return hardware_parameter(key, default_value);
          },
          &error_message)) {
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  imu_ = std::make_unique<mujoco_simulation::MujocoImus>();
  if (!imu_->configure(info_.sensors, *model, &error_message)) {
    RCLCPP_ERROR(logger, "%s", error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  if (joints_ != nullptr) {
    auto joint_interfaces = joints_->export_state_interfaces(info_.joints);
    interfaces.reserve(interfaces.size() + joint_interfaces.size());
    for (auto &interface : joint_interfaces) {
      interfaces.emplace_back(std::move(interface));
    }
  }
  if (imu_ != nullptr) {
    auto imu_interfaces = imu_->export_state_interfaces(info_.sensors);
    interfaces.reserve(interfaces.size() + imu_interfaces.size());
    for (auto &interface : imu_interfaces) {
      interfaces.emplace_back(std::move(interface));
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  if (joints_ == nullptr) {
    return {};
  }
  return joints_->export_command_interfaces(info_.joints);
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(
    const rclcpp_lifecycle::State &) {
  if (simulation_ != nullptr && !simulation_->is_running()) {
    simulation_->start();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State &) {
  if (simulation_ != nullptr) {
    simulation_->stop();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  simulation_->with_locked_data([this](const mjModel &, const mjData &data) {
    if (joints_ != nullptr) {
      joints_->read(data);
    }
    if (imu_ != nullptr) {
      imu_->read(data);
    }
  });
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  simulation_->with_locked_data([this](const mjModel &, mjData &data) {
    if (joints_ != nullptr) {
      joints_->write(data);
    }
  });
  return hardware_interface::return_type::OK;
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
