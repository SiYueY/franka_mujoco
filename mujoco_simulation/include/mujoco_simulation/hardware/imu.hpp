#pragma once

#include <mujoco/mujoco.h>

#include <hardware/data.hpp>
#include <string>

namespace mujoco_simulation {

struct ImuData {
  std::string name;

  std::string framequat_sensor_name;
  std::string gyro_sensor_name;
  std::string accelerometer_sensor_name;
};

struct ImuCommand {};

// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg
struct ImuState {
  Quaterniond orientation{0.0, 0.0, 0.0, 1.0};
  // Vector9d orientation_covariance;  // Row major about x, y, z axes
  Vector3d angular_velocity{0.0, 0.0, 0.0};
  // Vector9d angular_velocity_covariance; // Row major about x, y, z axes
  Vector3d linear_acceleration{0.0, 0.0, 0.0};
  // Vector9d linear_acceleration_covariance;  // Row major x, y z
};

class Imu : public HardwareInterface<ImuData, ImuCommand, ImuState> {
 public:
  Imu(const mjModel* model, mjData* data);
  ~Imu() override = default;
  ;

  bool init(const ImuData& data) override;
  bool reset() override;

  bool write(const ImuCommand& command) override;
  bool read(ImuState& state) override;

 private:
  const mjModel* model_{nullptr};
  mjData* mj_data_{nullptr};

  int framequat_address_{-1};
  int gyro_address_{-1};
  int accelerometer_address_{-1};

  ImuData data_;
};

}  // namespace mujoco_simulation
