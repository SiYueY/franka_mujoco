#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "franka_hardware/data.hpp"
#include "mujoco_simulation/hardware/camera.hpp"
#include "mujoco_simulation/hardware/imu.hpp"
#include "mujoco_simulation/hardware/joint.hpp"
#include "mujoco_simulation/hardware/lidar.hpp"
#include "mujoco_simulation/mujoco_simulation.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace franka_hardware {

class Robot {
 public:
  Robot();
  ~Robot();

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  Robot(Robot&&) = delete;
  Robot& operator=(Robot&&) = delete;

  bool initialize(const HardwareConfig& config);
  void start();
  void stop();
  bool is_running() const;

  bool configure_joint_mode(const std::string& joint_name,
                            mujoco_simulation::CommandInterfaceType mode);
  bool write_joint(const JointData& joint);

  bool read_joint(JointData* joint);
  bool read_imu(ImuData* imu);
  bool read_camera(CameraData* camera);
  bool read_lidar(LidarData* lidar);

  double simulation_time() const;
  const std::string& last_error() const;

  bool has_joint(const std::string& name) const;
  bool has_body(const std::string& name) const;
  bool has_site(const std::string& name) const;
  bool has_camera(const std::string& name) const;

  int joint_id(const std::string& name) const;
  int body_id(const std::string& name) const;
  int site_id(const std::string& name) const;
  int camera_id(const std::string& name) const;

  bool camera_fovy(const std::string& camera_name, double* fovy_degrees) const;
  bool fill_camera_info(const std::string& camera_name, int width, int height,
                        sensor_msgs::msg::CameraInfo* info) const;
  std::string resolve_frame_id(const std::string& configured_frame,
                               const std::string& fallback_name) const;

 private:
  bool register_joint(const JointData& joint);
  bool register_imu(const ImuData& imu);
  bool register_camera(const CameraData& camera);
  bool register_lidar(const LidarData& lidar);
  void set_last_error(std::string message) const;

  mujoco_simulation::SimulationConfig simulation_config_;
  std::unique_ptr<mujoco_simulation::MuJoCoSimulation> simulation_;
  mutable std::string last_error_;
};

}  // namespace franka_hardware
