#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_simulation/hardware/data.hpp"

namespace mujoco_simulation {

class Viewer;

enum class RenderMode {
  Headless,
  Viewer,
};

enum class SimulationJointCommandMode {
  None,
  Position,
  Velocity,
  Effort,
};

struct SimulationJointConfiguration {
  std::string name;
  std::string actuator_name;
  SimulationJointCommandMode command_mode{SimulationJointCommandMode::None};
};

struct SimulationJointCommand {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double effort{0.0};
};

struct SimulationJointState {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct SimulationImuConfiguration {
  std::string name;
  std::string framequat_sensor_name;
  std::string gyro_sensor_name;
  std::string accelerometer_sensor_name;
};

struct SimulationImuState {
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  double angular_velocity_x{0.0};
  double angular_velocity_y{0.0};
  double angular_velocity_z{0.0};
  double linear_acceleration_x{0.0};
  double linear_acceleration_y{0.0};
  double linear_acceleration_z{0.0};
};

struct SimulationImage {
  uint64_t timestamp{0};
  std::string frame_id;
  uint32_t height{0};
  uint32_t width{0};
  std::string encoding;
  uint8_t is_bigendian{0};
  uint32_t step{0};
  std::vector<uint8_t> data;
};

struct SimulationCameraInfo {
  uint32_t height{0};
  uint32_t width{0};
  std::string distortion_model;
  std::vector<double> d;
  Vector9d k{};
  Vector9d r{};
  Vector12d p{};
  uint32_t binning_x{0};
  uint32_t binning_y{0};
};

struct SimulationCameraConfiguration {
  std::string name;
  std::string camera_name;
  int height{0};
  int width{0};
  bool enable_rgb{true};
  bool enable_depth{false};
};

struct SimulationCameraState {
  SimulationImage image;
  SimulationImage depth_image;
  SimulationCameraInfo camera_info;
};

struct SimulationLaserScan {
  std::string frame_id;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double time_increment{0.0};
  double scan_time{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<double> ranges;
  std::vector<double> intensities;
};

struct SimulationLidarConfiguration {
  std::string name;
  std::string frame_name;
  std::string sensor_prefix;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

struct SimulationLidarState {
  SimulationLaserScan laser_scan;
};

struct SimulationConfig {
  std::string model_path;
  RenderMode render_mode = RenderMode::Headless;
  double sim_speed_factor = 1.0;
  std::string initial_keyframe;
};

class MuJoCoSimulation {
 public:
  MuJoCoSimulation();
  ~MuJoCoSimulation();

  MuJoCoSimulation(const MuJoCoSimulation &) = delete;
  MuJoCoSimulation &operator=(const MuJoCoSimulation &) = delete;

  bool initialize(const SimulationConfig &config, std::string *error_message = nullptr);
  void start();
  void stop();

  bool set_paused(bool paused);
  bool paused() const;
  bool reset(const std::string &keyframe, std::string *error_message = nullptr);
  bool step(uint32_t steps, std::string *error_message = nullptr);

  bool register_joint(const SimulationJointConfiguration &configuration,
                      std::string *error_message = nullptr);
  bool configure_joint_command_mode(const std::string &joint_name,
                                    SimulationJointCommandMode command_mode,
                                    std::string *error_message = nullptr);
  bool write_joint(const SimulationJointCommand &command, std::string *error_message = nullptr);
  bool read_joint(const std::string &joint_name, SimulationJointState *state,
                  std::string *error_message = nullptr);

  bool register_imu(const SimulationImuConfiguration &configuration,
                    std::string *error_message = nullptr);
  bool read_imu(const std::string &imu_name, SimulationImuState *state,
                std::string *error_message = nullptr);

  bool register_camera(const SimulationCameraConfiguration &configuration,
                       std::string *error_message = nullptr);
  bool read_camera(const std::string &camera_name, SimulationCameraState *state,
                   std::string *error_message = nullptr);

  bool register_lidar(const SimulationLidarConfiguration &configuration,
                      std::string *error_message = nullptr);
  bool read_lidar(const std::string &lidar_name, SimulationLidarState *state,
                  std::string *error_message = nullptr);

  uint64_t step_count() const;
  double simulation_time() const;
  bool has_joint(const std::string &joint_name) const;
  bool has_body(const std::string &body_name) const;
  bool has_site(const std::string &site_name) const;
  bool has_camera(const std::string &camera_name) const;
  int joint_id(const std::string &joint_name) const;
  int body_id(const std::string &body_name) const;
  int site_id(const std::string &site_name) const;
  int camera_id(const std::string &camera_name) const;
  bool camera_fovy(const std::string &camera_name, double *fovy_degrees,
                   std::string *error_message = nullptr) const;
  const mjModel *model() const;
  bool is_initialized() const;
  bool is_running() const;
  const SimulationConfig &config() const;

  void with_locked_data(const std::function<void(const mjModel &, mjData &)> &callback);
  void with_locked_data(const std::function<void(const mjModel &, const mjData &)> &callback) const;
  bool copy_data_to(mjData *dest) const;

 private:
  void physics_loop();
  bool load_model(const std::string &model_path, std::string *error_message);
  bool start_viewer(std::string *error_message);
  int keyframe_id(const std::string &keyframe) const;

  class Impl;

  SimulationConfig config_;
  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  std::unique_ptr<Impl> impl_;
  std::unique_ptr<Viewer> viewer_;

  mutable std::mutex mutex_;
  std::thread physics_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool paused_{false};
  std::atomic_uint64_t step_count_{0};
};

RenderMode parse_render_mode(const std::string &value);
const char *to_string(RenderMode mode);

}  // namespace mujoco_simulation
