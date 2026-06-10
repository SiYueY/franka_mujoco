#include "mujoco_simulation/mujoco_simulation.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>

#include "mujoco_simulation/hardware/hardware_manager.hpp"
#include "mujoco_simulation/viewer/viewer.hpp"

namespace mujoco_simulation {
namespace {
constexpr int kLoadErrorLength = 1024;

int model_name_to_id(const mjModel* model, int object_type, const std::string& name) {
  if (model == nullptr || name.empty()) {
    return -1;
  }
  return mj_name2id(model, object_type, name.c_str());
}

CommandInterfaceType to_backend_command_mode(SimulationJointCommandMode mode) {
  switch (mode) {
    case SimulationJointCommandMode::None:
      return CommandInterfaceType::None;
    case SimulationJointCommandMode::Position:
      return CommandInterfaceType::Position;
    case SimulationJointCommandMode::Velocity:
      return CommandInterfaceType::Velocity;
    case SimulationJointCommandMode::Effort:
      return CommandInterfaceType::Effort;
  }
  return CommandInterfaceType::None;
}

SimulationJointState to_public_joint_state(const mujoco_simulation::JointState& state) {
  SimulationJointState public_state;
  public_state.name = state.name;
  public_state.position = state.position;
  public_state.velocity = state.velocity;
  public_state.effort = state.effort;
  return public_state;
}

SimulationImuState to_public_imu_state(const mujoco_simulation::ImuState& state) {
  SimulationImuState public_state;
  public_state.orientation_x = state.orientation[0];
  public_state.orientation_y = state.orientation[1];
  public_state.orientation_z = state.orientation[2];
  public_state.orientation_w = state.orientation[3];
  public_state.angular_velocity_x = state.angular_velocity[0];
  public_state.angular_velocity_y = state.angular_velocity[1];
  public_state.angular_velocity_z = state.angular_velocity[2];
  public_state.linear_acceleration_x = state.linear_acceleration[0];
  public_state.linear_acceleration_y = state.linear_acceleration[1];
  public_state.linear_acceleration_z = state.linear_acceleration[2];
  return public_state;
}

SimulationCameraState to_public_camera_state(const mujoco_simulation::CameraState& state) {
  SimulationCameraState public_state;
  public_state.image.timestamp = state.image.timestamp;
  public_state.image.frame_id = state.image.frame_id;
  public_state.image.height = state.image.height;
  public_state.image.width = state.image.width;
  public_state.image.encoding = state.image.encoding;
  public_state.image.is_bigendian = state.image.is_bigendian;
  public_state.image.step = state.image.step;
  public_state.image.data = state.image.data;

  public_state.depth_image.timestamp = state.depth_image.timestamp;
  public_state.depth_image.frame_id = state.depth_image.frame_id;
  public_state.depth_image.height = state.depth_image.height;
  public_state.depth_image.width = state.depth_image.width;
  public_state.depth_image.encoding = state.depth_image.encoding;
  public_state.depth_image.is_bigendian = state.depth_image.is_bigendian;
  public_state.depth_image.step = state.depth_image.step;
  public_state.depth_image.data = state.depth_image.data;

  public_state.camera_info.height = state.camera_info.height;
  public_state.camera_info.width = state.camera_info.width;
  public_state.camera_info.distortion_model = state.camera_info.distortion_model;
  public_state.camera_info.d = state.camera_info.d;
  public_state.camera_info.k = state.camera_info.k;
  public_state.camera_info.r = state.camera_info.r;
  public_state.camera_info.p = state.camera_info.p;
  public_state.camera_info.binning_x = state.camera_info.binning_x;
  public_state.camera_info.binning_y = state.camera_info.binning_y;
  return public_state;
}

SimulationLidarState to_public_lidar_state(const mujoco_simulation::LidarState& state) {
  SimulationLidarState public_state;
  public_state.laser_scan.frame_id = state.laser_scan.frame_id;
  public_state.laser_scan.angle_min = state.laser_scan.angle_min;
  public_state.laser_scan.angle_max = state.laser_scan.angle_max;
  public_state.laser_scan.angle_increment = state.laser_scan.angle_increment;
  public_state.laser_scan.time_increment = state.laser_scan.time_increment;
  public_state.laser_scan.scan_time = state.laser_scan.scan_time;
  public_state.laser_scan.range_min = state.laser_scan.range_min;
  public_state.laser_scan.range_max = state.laser_scan.range_max;
  public_state.laser_scan.ranges = state.laser_scan.ranges;
  public_state.laser_scan.intensities = state.laser_scan.intensities;
  return public_state;
}
}  // namespace

class MuJoCoSimulation::Impl {
 public:
  std::unique_ptr<HardwareManager> hardware_manager;
  std::unique_ptr<HardwareManager> render_hardware_manager;
  std::map<std::string, SimulationJointConfiguration> joints;
  std::map<std::string, SimulationImuConfiguration> imus;
  std::map<std::string, SimulationCameraConfiguration> cameras;
  std::map<std::string, SimulationLidarConfiguration> lidars;
};

MuJoCoSimulation::MuJoCoSimulation() : impl_(std::make_unique<Impl>()) {}

MuJoCoSimulation::~MuJoCoSimulation() {
  stop();

  std::lock_guard<std::mutex> lock(mutex_);
  if (data_ != nullptr) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

bool MuJoCoSimulation::initialize(const SimulationConfig &config, std::string *error_message) {
  if (is_initialized()) {
    if (error_message != nullptr) {
      *error_message = "MuJoCoSimulation is already initialized.";
    }
    return false;
  }

  config_ = config;

  if (!load_model(config.model_path, error_message)) {
    return false;
  }

  if (config.render_mode == RenderMode::Viewer && !start_viewer(error_message)) {
    return false;
  }

  if (!config.initial_keyframe.empty() && !reset(config.initial_keyframe, error_message)) {
    return false;
  }

  return true;
}

void MuJoCoSimulation::start() {
  if (running_.exchange(true)) {
    return;
  }
  physics_thread_ = std::thread([this]() { physics_loop(); });
}

void MuJoCoSimulation::stop() {
  running_.store(false);
  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }
  if (viewer_ != nullptr) {
    viewer_->stop();
  }
}

bool MuJoCoSimulation::set_paused(bool paused) {
  paused_.store(paused);
  return true;
}

bool MuJoCoSimulation::paused() const { return paused_.load(); }

bool MuJoCoSimulation::reset(const std::string &keyframe, std::string *error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr || data_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo model is not loaded.";
    }
    return false;
  }

  if (keyframe.empty()) {
    mj_resetData(model_, data_);
  } else {
    const int id = keyframe_id(keyframe);
    if (id < 0) {
      if (error_message != nullptr) {
        *error_message = "MuJoCo keyframe not found: " + keyframe;
      }
      return false;
    }
    mj_resetDataKeyframe(model_, data_, id);
  }

  mj_forward(model_, data_);
  step_count_.store(0);
  return true;
}

bool MuJoCoSimulation::step(uint32_t steps, std::string *error_message) {
  if (steps == 0) {
    if (error_message != nullptr) {
      *error_message = "Step count must be greater than zero.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr || data_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo model is not loaded.";
    }
    return false;
  }

  for (uint32_t i = 0; i < steps; ++i) {
    mj_step(model_, data_);
    ++step_count_;
  }
  if (viewer_ != nullptr && !viewer_->sync()) {
    if (error_message != nullptr) {
      *error_message = "Failed to sync MuJoCo viewer after stepping.";
    }
    return false;
  }
  return true;
}

bool MuJoCoSimulation::register_joint(const SimulationJointConfiguration& configuration,
                                      std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  JointInfo data;
  data.name = configuration.name;
  data.actuator_name = configuration.actuator_name;
  data.command_mode = to_backend_command_mode(configuration.command_mode);
  if (!impl_->hardware_manager->register_joint(data)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }

  impl_->joints[configuration.name] = configuration;
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::configure_joint_command_mode(const std::string& joint_name,
                                                    SimulationJointCommandMode command_mode,
                                                    std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  const auto it = impl_->joints.find(joint_name);
  if (it == impl_->joints.end()) {
    if (error_message != nullptr) {
      *error_message = "Joint is not registered: " + joint_name;
    }
    return false;
  }

  if (!impl_->hardware_manager->unregister_joint(joint_name)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }

  SimulationJointConfiguration updated = it->second;
  updated.command_mode = command_mode;
  JointInfo data;
  data.name = updated.name;
  data.actuator_name = updated.actuator_name;
  data.command_mode = to_backend_command_mode(updated.command_mode);
  if (!impl_->hardware_manager->register_joint(data)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }

  it->second = updated;
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::write_joint(const SimulationJointCommand& command,
                                   std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  mujoco_simulation::JointCommand backend_command;
  backend_command.name = command.name;
  backend_command.position = command.position;
  backend_command.velocity = command.velocity;
  backend_command.acceleration = command.acceleration;
  backend_command.effort = command.effort;
  if (!impl_->hardware_manager->write_joint(command.name, backend_command)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::read_joint(const std::string& joint_name, SimulationJointState* state,
                                  std::string* error_message) {
  if (state == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Joint state output pointer must not be null.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  mujoco_simulation::JointState backend_state;
  if (!impl_->hardware_manager->read_joint(joint_name, backend_state)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }
  *state = to_public_joint_state(backend_state);
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::register_imu(const SimulationImuConfiguration& configuration,
                                    std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  ImuInfo data;
  data.name = configuration.name;
  data.framequat_sensor_name = configuration.framequat_sensor_name;
  data.gyro_sensor_name = configuration.gyro_sensor_name;
  data.accelerometer_sensor_name = configuration.accelerometer_sensor_name;
  if (!impl_->hardware_manager->register_imu(data)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }

  impl_->imus[configuration.name] = configuration;
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::read_imu(const std::string& imu_name, SimulationImuState* state,
                                std::string* error_message) {
  if (state == nullptr) {
    if (error_message != nullptr) {
      *error_message = "IMU state output pointer must not be null.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  mujoco_simulation::ImuState backend_state;
  if (!impl_->hardware_manager->read_imu(imu_name, backend_state)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }
  *state = to_public_imu_state(backend_state);
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::register_camera(const SimulationCameraConfiguration& configuration,
                                       std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (configuration.name.empty()) {
    if (error_message != nullptr) {
      *error_message = "Camera name must not be empty.";
    }
    return false;
  }
  if (impl_->cameras.find(configuration.name) != impl_->cameras.end()) {
    if (error_message != nullptr) {
      *error_message = "Camera already registered: " + configuration.name;
    }
    return false;
  }

  impl_->cameras[configuration.name] = configuration;

  if (viewer_ != nullptr && viewer_->scene() != nullptr && viewer_->render_context() != nullptr) {
    if (impl_->render_hardware_manager == nullptr) {
      impl_->render_hardware_manager = std::make_unique<HardwareManager>(
          model_, data_, viewer_->scene(), viewer_->render_context());
    }

    CameraSpec data;
    data.name = configuration.name;
    data.camera_name = configuration.camera_name;
    data.height = configuration.height;
    data.width = configuration.width;
    data.enable_rgb = configuration.enable_rgb;
    data.enable_depth = configuration.enable_depth;
    if (!impl_->render_hardware_manager->register_camera(data)) {
      if (error_message != nullptr) {
        *error_message = impl_->render_hardware_manager->last_error();
      }
      return false;
    }
  }

  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::read_camera(const std::string& camera_name, SimulationCameraState* state,
                                   std::string* error_message) {
  if (state == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Camera state output pointer must not be null.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (viewer_ == nullptr || viewer_->scene() == nullptr || viewer_->render_context() == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Camera access requires render_mode=viewer.";
    }
    return false;
  }
  if (impl_->render_hardware_manager == nullptr) {
    impl_->render_hardware_manager = std::make_unique<HardwareManager>(
        model_, data_, viewer_->scene(), viewer_->render_context());
    for (const auto& [name, configuration] : impl_->cameras) {
      CameraSpec data;
      data.name = configuration.name;
      data.camera_name = configuration.camera_name;
      data.height = configuration.height;
      data.width = configuration.width;
      data.enable_rgb = configuration.enable_rgb;
      data.enable_depth = configuration.enable_depth;
      if (!impl_->render_hardware_manager->register_camera(data)) {
        if (error_message != nullptr) {
          *error_message = impl_->render_hardware_manager->last_error();
        }
        return false;
      }
    }
  }

  mujoco_simulation::CameraState backend_state;
  if (!impl_->render_hardware_manager->read_camera(camera_name, backend_state)) {
    if (error_message != nullptr) {
      *error_message = impl_->render_hardware_manager->last_error();
    }
    return false;
  }
  *state = to_public_camera_state(backend_state);
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::register_lidar(const SimulationLidarConfiguration& configuration,
                                      std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  LidarInfo data;
  data.name = configuration.name;
  data.frame_name = configuration.frame_name;
  data.sensor_prefix = configuration.sensor_prefix;
  data.angle_min = configuration.angle_min;
  data.angle_max = configuration.angle_max;
  data.angle_increment = configuration.angle_increment;
  data.range_min = configuration.range_min;
  data.range_max = configuration.range_max;
  if (!impl_->hardware_manager->register_lidar(data)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }

  impl_->lidars[configuration.name] = configuration;
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

bool MuJoCoSimulation::read_lidar(const std::string& lidar_name, SimulationLidarState* state,
                                  std::string* error_message) {
  if (state == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Lidar state output pointer must not be null.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_->hardware_manager == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo hardware manager is not initialized.";
    }
    return false;
  }

  mujoco_simulation::LidarState backend_state;
  if (!impl_->hardware_manager->read_lidar(lidar_name, backend_state)) {
    if (error_message != nullptr) {
      *error_message = impl_->hardware_manager->last_error();
    }
    return false;
  }
  *state = to_public_lidar_state(backend_state);
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

uint64_t MuJoCoSimulation::step_count() const { return step_count_.load(); }

double MuJoCoSimulation::simulation_time() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_ == nullptr) {
    return 0.0;
  }
  return data_->time;
}

bool MuJoCoSimulation::has_joint(const std::string& joint_name) const {
  return joint_id(joint_name) >= 0;
}

bool MuJoCoSimulation::has_body(const std::string& body_name) const {
  return body_id(body_name) >= 0;
}

bool MuJoCoSimulation::has_site(const std::string& site_name) const {
  return site_id(site_name) >= 0;
}

bool MuJoCoSimulation::has_camera(const std::string& camera_name) const {
  return camera_id(camera_name) >= 0;
}

int MuJoCoSimulation::joint_id(const std::string& joint_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_JOINT, joint_name);
}

int MuJoCoSimulation::body_id(const std::string& body_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_BODY, body_name);
}

int MuJoCoSimulation::site_id(const std::string& site_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_SITE, site_name);
}

int MuJoCoSimulation::camera_id(const std::string& camera_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_CAMERA, camera_name);
}

bool MuJoCoSimulation::camera_fovy(const std::string& camera_name, double* fovy_degrees,
                                   std::string* error_message) const {
  if (fovy_degrees == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Camera fovy output pointer must not be null.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo model is not loaded.";
    }
    return false;
  }

  const int camera_id = model_name_to_id(model_, mjOBJ_CAMERA, camera_name);
  if (camera_id < 0) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo camera not found: " + camera_name;
    }
    return false;
  }

  *fovy_degrees = static_cast<double>(model_->cam_fovy[camera_id]);
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

const mjModel *MuJoCoSimulation::model() const { return model_; }

bool MuJoCoSimulation::is_initialized() const { return model_ != nullptr && data_ != nullptr; }

bool MuJoCoSimulation::is_running() const { return running_.load(); }

const SimulationConfig &MuJoCoSimulation::config() const { return config_; }

void MuJoCoSimulation::with_locked_data(
    const std::function<void(const mjModel &, mjData &)> &callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ != nullptr && data_ != nullptr) {
    callback(*model_, *data_);
  }
}

void MuJoCoSimulation::with_locked_data(
    const std::function<void(const mjModel &, const mjData &)> &callback) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ != nullptr && data_ != nullptr) {
    callback(*model_, *data_);
  }
}

bool MuJoCoSimulation::copy_data_to(mjData *dest) const {
  if (dest == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr || data_ == nullptr) {
    return false;
  }
  mj_copyData(dest, model_, data_);
  return true;
}

void MuJoCoSimulation::physics_loop() {
  using clock = std::chrono::steady_clock;
  auto next_step = clock::now();

  while (running_.load()) {
    double timestep = 0.001;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (model_ != nullptr) {
        timestep = model_->opt.timestep;
      }
    }

    const double speed = config_.sim_speed_factor > 0.0 ? config_.sim_speed_factor : 1.0;
    const auto period = std::chrono::duration<double>(timestep / speed);
    next_step += std::chrono::duration_cast<clock::duration>(period);

    if (!paused_.load()) {
      std::string ignored;
      step(1, &ignored);
    }

    std::this_thread::sleep_until(next_step);
    if (next_step < clock::now() - std::chrono::seconds(1)) {
      next_step = clock::now();
    }
  }
}

bool MuJoCoSimulation::load_model(const std::string &model_path, std::string *error_message) {
  if (model_path.empty()) {
    if (error_message != nullptr) {
      *error_message = "mujoco_model_path must not be empty.";
    }
    return false;
  }

  char load_error[kLoadErrorLength] = {0};
  mjModel *new_model = mj_loadXML(model_path.c_str(), nullptr, load_error, kLoadErrorLength);
  if (new_model == nullptr) {
    if (error_message != nullptr) {
      *error_message = std::string("Failed to load MuJoCo model: ") + load_error;
    }
    return false;
  }

  mjData *new_data = mj_makeData(new_model);
  if (new_data == nullptr) {
    mj_deleteModel(new_model);
    if (error_message != nullptr) {
      *error_message = "Failed to allocate MuJoCo data.";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
  model_ = new_model;
  data_ = new_data;
  impl_->hardware_manager = std::make_unique<HardwareManager>(model_, data_);
  impl_->render_hardware_manager.reset();
  impl_->joints.clear();
  impl_->imus.clear();
  impl_->cameras.clear();
  impl_->lidars.clear();
  mj_forward(model_, data_);
  return true;
}

bool MuJoCoSimulation::start_viewer(std::string *error_message) {
  viewer_ = std::make_unique<Viewer>();
  if (!viewer_->initialize(error_message)) {
    viewer_.reset();
    return false;
  }
  if (!viewer_->load(model_, data_, config_.model_path, error_message)) {
    viewer_.reset();
    return false;
  }
  if (!viewer_->start(error_message)) {
    viewer_.reset();
    return false;
  }
  return true;
}

int MuJoCoSimulation::keyframe_id(const std::string &keyframe) const {
  return mj_name2id(model_, mjOBJ_KEY, keyframe.c_str());
}

RenderMode parse_render_mode(const std::string &value) {
  if (value == "headless") {
    return RenderMode::Headless;
  }
  if (value == "viewer") {
    return RenderMode::Viewer;
  }
  throw std::invalid_argument("render_mode must be 'headless' or 'viewer'.");
}

const char *to_string(RenderMode mode) {
  switch (mode) {
    case RenderMode::Headless:
      return "headless";
    case RenderMode::Viewer:
      return "viewer";
  }
  return "unknown";
}

}  // namespace mujoco_simulation
