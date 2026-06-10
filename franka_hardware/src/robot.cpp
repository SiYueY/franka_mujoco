#include "franka_hardware/robot.hpp"

#include <cmath>

namespace franka_hardware {
namespace {

mujoco_simulation::SimulationJointCommandMode to_simulation_joint_mode(
    mujoco_simulation::CommandInterfaceType mode) {
  switch (mode) {
    case mujoco_simulation::CommandInterfaceType::Position:
      return mujoco_simulation::SimulationJointCommandMode::Position;
    case mujoco_simulation::CommandInterfaceType::Velocity:
      return mujoco_simulation::SimulationJointCommandMode::Velocity;
    case mujoco_simulation::CommandInterfaceType::Effort:
      return mujoco_simulation::SimulationJointCommandMode::Effort;
    case mujoco_simulation::CommandInterfaceType::None:
    default:
      return mujoco_simulation::SimulationJointCommandMode::None;
  }
}

}  // namespace

Robot::Robot() = default;

Robot::~Robot() = default;

bool Robot::initialize(const HardwareConfig& config) {
  simulation_config_ = config.simulation;
  simulation_ = std::make_unique<mujoco_simulation::MuJoCoSimulation>();
  if (!simulation_->initialize(simulation_config_, &last_error_)) {
    return false;
  }
  for (const auto& joint : config.joints) {
    if (!register_joint(joint)) {
      return false;
    }
  }
  for (const auto& imu : config.imus) {
    if (!register_imu(imu)) {
      return false;
    }
  }
  for (const auto& camera : config.cameras) {
    if (!register_camera(camera)) {
      return false;
    }
  }
  for (const auto& lidar : config.lidars) {
    if (!register_lidar(lidar)) {
      return false;
    }
  }
  last_error_.clear();
  return true;
}

void Robot::start() {
  if (simulation_ != nullptr && !simulation_->is_running()) {
    simulation_->start();
  }
}

void Robot::stop() {
  if (simulation_ != nullptr && simulation_->is_running()) {
    simulation_->stop();
  }
}

bool Robot::is_running() const { return simulation_ != nullptr && simulation_->is_running(); }

bool Robot::register_joint(const JointData& joint) {
  mujoco_simulation::SimulationJointConfiguration configuration;
  configuration.name = joint.info.name;
  configuration.actuator_name = joint.info.actuator_name;
  configuration.command_mode = to_simulation_joint_mode(joint.info.command_mode);
  return simulation_->register_joint(configuration, &last_error_);
}

bool Robot::register_imu(const ImuData& imu) {
  mujoco_simulation::SimulationImuConfiguration configuration;
  configuration.name = imu.info.name;
  configuration.framequat_sensor_name = imu.info.framequat_sensor_name;
  configuration.gyro_sensor_name = imu.info.gyro_sensor_name;
  configuration.accelerometer_sensor_name = imu.info.accelerometer_sensor_name;
  return simulation_->register_imu(configuration, &last_error_);
}

bool Robot::register_camera(const CameraData& camera) {
  mujoco_simulation::SimulationCameraConfiguration configuration;
  configuration.name = camera.info.name;
  configuration.camera_name = camera.info.camera_name;
  configuration.width = camera.info.width;
  configuration.height = camera.info.height;
  configuration.enable_rgb = camera.info.enable_rgb;
  configuration.enable_depth = camera.info.enable_depth;
  return simulation_->register_camera(configuration, &last_error_);
}

bool Robot::register_lidar(const LidarData& lidar) {
  mujoco_simulation::SimulationLidarConfiguration configuration;
  configuration.name = lidar.info.name;
  configuration.frame_name = lidar.info.frame_name;
  configuration.sensor_prefix = lidar.info.sensor_prefix;
  configuration.angle_min = lidar.info.angle_min;
  configuration.angle_max = lidar.info.angle_max;
  configuration.angle_increment = lidar.info.angle_increment;
  configuration.range_min = lidar.info.range_min;
  configuration.range_max = lidar.info.range_max;
  return simulation_->register_lidar(configuration, &last_error_);
}

bool Robot::configure_joint_mode(const std::string& joint_name,
                                 mujoco_simulation::CommandInterfaceType mode) {
  return simulation_->configure_joint_command_mode(joint_name, to_simulation_joint_mode(mode),
                                                   &last_error_);
}

bool Robot::write_joint(const JointData& joint) {
  mujoco_simulation::SimulationJointCommand simulation_command;
  simulation_command.name = joint.name;
  simulation_command.position = joint.command.position;
  simulation_command.velocity = joint.command.velocity;
  simulation_command.acceleration = joint.command.acceleration;
  simulation_command.effort = joint.command.effort;
  const bool ok = simulation_->write_joint(simulation_command, &last_error_);
  if (ok) {
    last_error_.clear();
  }
  return ok;
}

bool Robot::read_joint(JointData* joint) {
  if (joint == nullptr) {
    set_last_error("JointData pointer must not be null.");
    return false;
  }
  mujoco_simulation::SimulationJointState simulation_state;
  if (!simulation_->read_joint(joint->name, &simulation_state, &last_error_)) {
    return false;
  }
  joint->state.name = simulation_state.name;
  joint->state.position = simulation_state.position;
  joint->state.velocity = simulation_state.velocity;
  joint->state.effort = simulation_state.effort;
  last_error_.clear();
  return true;
}

bool Robot::read_imu(ImuData* imu) {
  if (imu == nullptr) {
    set_last_error("ImuData pointer must not be null.");
    return false;
  }
  mujoco_simulation::SimulationImuState simulation_state;
  if (!simulation_->read_imu(imu->name, &simulation_state, &last_error_)) {
    return false;
  }
  imu->state.orientation = {simulation_state.orientation_x, simulation_state.orientation_y,
                            simulation_state.orientation_z, simulation_state.orientation_w};
  imu->state.angular_velocity = {simulation_state.angular_velocity_x,
                                 simulation_state.angular_velocity_y,
                                 simulation_state.angular_velocity_z};
  imu->state.linear_acceleration = {simulation_state.linear_acceleration_x,
                                    simulation_state.linear_acceleration_y,
                                    simulation_state.linear_acceleration_z};
  last_error_.clear();
  return true;
}

bool Robot::read_camera(CameraData* camera) {
  if (camera == nullptr) {
    set_last_error("CameraData pointer must not be null.");
    return false;
  }
  mujoco_simulation::SimulationCameraState simulation_state;
  if (!simulation_->read_camera(camera->name, &simulation_state, &last_error_)) {
    return false;
  }

  camera->state.image.timestamp = simulation_state.image.timestamp;
  camera->state.image.frame_id = simulation_state.image.frame_id;
  camera->state.image.height = simulation_state.image.height;
  camera->state.image.width = simulation_state.image.width;
  camera->state.image.encoding = simulation_state.image.encoding;
  camera->state.image.is_bigendian = simulation_state.image.is_bigendian;
  camera->state.image.step = simulation_state.image.step;
  camera->state.image.data = simulation_state.image.data;

  camera->state.depth_image.timestamp = simulation_state.depth_image.timestamp;
  camera->state.depth_image.frame_id = simulation_state.depth_image.frame_id;
  camera->state.depth_image.height = simulation_state.depth_image.height;
  camera->state.depth_image.width = simulation_state.depth_image.width;
  camera->state.depth_image.encoding = simulation_state.depth_image.encoding;
  camera->state.depth_image.is_bigendian = simulation_state.depth_image.is_bigendian;
  camera->state.depth_image.step = simulation_state.depth_image.step;
  camera->state.depth_image.data = simulation_state.depth_image.data;

  camera->state.camera_info.height = simulation_state.camera_info.height;
  camera->state.camera_info.width = simulation_state.camera_info.width;
  camera->state.camera_info.distortion_model = simulation_state.camera_info.distortion_model;
  camera->state.camera_info.d = simulation_state.camera_info.d;
  camera->state.camera_info.k = simulation_state.camera_info.k;
  camera->state.camera_info.r = simulation_state.camera_info.r;
  camera->state.camera_info.p = simulation_state.camera_info.p;
  camera->state.camera_info.binning_x = simulation_state.camera_info.binning_x;
  camera->state.camera_info.binning_y = simulation_state.camera_info.binning_y;
  last_error_.clear();
  return true;
}

bool Robot::read_lidar(LidarData* lidar) {
  if (lidar == nullptr) {
    set_last_error("LidarData pointer must not be null.");
    return false;
  }
  mujoco_simulation::SimulationLidarState simulation_state;
  if (!simulation_->read_lidar(lidar->name, &simulation_state, &last_error_)) {
    return false;
  }
  lidar->state.laser_scan.frame_id = simulation_state.laser_scan.frame_id;
  lidar->state.laser_scan.angle_min = simulation_state.laser_scan.angle_min;
  lidar->state.laser_scan.angle_max = simulation_state.laser_scan.angle_max;
  lidar->state.laser_scan.angle_increment = simulation_state.laser_scan.angle_increment;
  lidar->state.laser_scan.time_increment = simulation_state.laser_scan.time_increment;
  lidar->state.laser_scan.scan_time = simulation_state.laser_scan.scan_time;
  lidar->state.laser_scan.range_min = simulation_state.laser_scan.range_min;
  lidar->state.laser_scan.range_max = simulation_state.laser_scan.range_max;
  lidar->state.laser_scan.ranges = simulation_state.laser_scan.ranges;
  lidar->state.laser_scan.intensities = simulation_state.laser_scan.intensities;
  last_error_.clear();
  return true;
}

double Robot::simulation_time() const {
  return simulation_ == nullptr ? 0.0 : simulation_->simulation_time();
}

const std::string& Robot::last_error() const { return last_error_; }

bool Robot::has_joint(const std::string& name) const { return joint_id(name) >= 0; }
bool Robot::has_body(const std::string& name) const { return body_id(name) >= 0; }
bool Robot::has_site(const std::string& name) const { return site_id(name) >= 0; }
bool Robot::has_camera(const std::string& name) const { return camera_id(name) >= 0; }

int Robot::joint_id(const std::string& name) const {
  return simulation_ == nullptr ? -1 : simulation_->joint_id(name);
}

int Robot::body_id(const std::string& name) const {
  return simulation_ == nullptr ? -1 : simulation_->body_id(name);
}

int Robot::site_id(const std::string& name) const {
  return simulation_ == nullptr ? -1 : simulation_->site_id(name);
}

int Robot::camera_id(const std::string& name) const {
  return simulation_ == nullptr ? -1 : simulation_->camera_id(name);
}

bool Robot::camera_fovy(const std::string& camera_name, double* fovy_degrees) const {
  if (simulation_ == nullptr) {
    set_last_error("MuJoCo simulation is not available.");
    return false;
  }
  const bool ok = simulation_->camera_fovy(camera_name, fovy_degrees, &last_error_);
  if (ok) {
    last_error_.clear();
  }
  return ok;
}

bool Robot::fill_camera_info(const std::string& camera_name, int width, int height,
                             sensor_msgs::msg::CameraInfo* info) const {
  if (info == nullptr) {
    set_last_error("CameraInfo output pointer must not be null.");
    return false;
  }
  if (width <= 0 || height <= 0) {
    set_last_error("Camera dimensions must be positive.");
    return false;
  }

  double fovy_degrees = 0.0;
  if (!camera_fovy(camera_name, &fovy_degrees)) {
    return false;
  }

  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const double fovy_radians = fovy_degrees * M_PI / 180.0;
  const double fy = static_cast<double>(height) / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians = 2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  const double fx = static_cast<double>(width) / (2.0 * std::tan(fovx_radians / 2.0));
  const double cx = (static_cast<double>(width) - 1.0) / 2.0;
  const double cy = (static_cast<double>(height) - 1.0) / 2.0;
  info->width = static_cast<uint32_t>(width);
  info->height = static_cast<uint32_t>(height);
  info->distortion_model = "plumb_bob";
  info->d.assign(5, 0.0);
  info->k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info->p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  last_error_.clear();
  return true;
}

std::string Robot::resolve_frame_id(const std::string& configured_frame,
                                    const std::string& fallback_name) const {
  return configured_frame.empty() ? fallback_name : configured_frame;
}

void Robot::set_last_error(std::string message) const { last_error_ = std::move(message); }

}  // namespace franka_hardware
