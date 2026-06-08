#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <hardware_interface/hardware_info.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "mujoco_simulation/mujoco_imu.hpp"
#include "mujoco_simulation/mujoco_joint.hpp"

namespace {

std::string write_file(const std::string& name, const std::string& contents) {
  const std::string path = "/tmp/" + name;
  std::ofstream out(path);
  out << contents;
  out.close();
  return path;
}

hardware_interface::InterfaceInfo make_interface(const std::string& name) {
  hardware_interface::InterfaceInfo info;
  info.name = name;
  return info;
}

hardware_interface::ComponentInfo make_joint(const std::string& name,
                                             const std::vector<std::string>& command_interfaces,
                                             const std::vector<std::string>& state_interfaces) {
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  for (const auto& command : command_interfaces) {
    joint.command_interfaces.push_back(make_interface(command));
  }
  for (const auto& state : state_interfaces) {
    joint.state_interfaces.push_back(make_interface(state));
  }
  return joint;
}

hardware_interface::ComponentInfo make_sensor(
    const std::string& name, const std::unordered_map<std::string, std::string>& parameters,
    const std::vector<std::string>& state_interfaces = {}) {
  hardware_interface::ComponentInfo sensor;
  sensor.name = name;
  sensor.type = "sensor";
  sensor.parameters = parameters;
  for (const auto& state : state_interfaces) {
    sensor.state_interfaces.push_back(make_interface(state));
  }
  return sensor;
}

hardware_interface::HardwareInfo make_hardware_info(
    const std::vector<hardware_interface::ComponentInfo>& joints,
    const std::vector<hardware_interface::ComponentInfo>& sensors = {},
    const std::unordered_map<std::string, std::string>& hardware_params = {}) {
  hardware_interface::HardwareInfo info;
  info.name = "TestSystem";
  info.type = "system";
  info.hardware_class_type = "mujoco_ros2_bridge/MujocoSystemInterface";
  info.hardware_parameters = hardware_params;
  info.joints = joints;
  info.sensors = sensors;
  return info;
}

std::vector<std::string> command_names(
    const std::vector<hardware_interface::CommandInterface>& interfaces) {
  std::vector<std::string> result;
  for (const auto& interface : interfaces) {
    result.push_back(interface.get_name());
  }
  return result;
}

std::vector<std::string> state_names(
    const std::vector<hardware_interface::StateInterface>& interfaces) {
  std::vector<std::string> result;
  for (const auto& interface : interfaces) {
    result.push_back(interface.get_name());
  }
  return result;
}

double state_value(const std::vector<hardware_interface::StateInterface>& interfaces,
                   const std::string& name, const std::string& interface_name) {
  const auto it = std::find_if(interfaces.begin(), interfaces.end(), [&](const auto& interface) {
    return interface.get_prefix_name() == name && interface.get_interface_name() == interface_name;
  });
  if (it == interfaces.end()) {
    throw std::runtime_error("State interface not found.");
  }
  return it->get_value();
}

struct LoadedModel {
  explicit LoadedModel(const std::string& model_path) {
    char error[1024] = {0};
    model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr) {
      throw std::runtime_error(error[0] == '\0' ? "Failed to load MJCF." : error);
    }
    data = mj_makeData(model);
    if (data == nullptr) {
      mj_deleteModel(model);
      throw std::runtime_error("Failed to create mjData.");
    }
  }

  ~LoadedModel() {
    if (data != nullptr) {
      mj_deleteData(data);
    }
    if (model != nullptr) {
      mj_deleteModel(model);
    }
  }

  mjModel* model = nullptr;
  mjData* data = nullptr;
};

}  // namespace

TEST(MujocoJointsTest, SupportsMixedMobileBaseAndPassiveJoints) {
  const std::string model_path =
      write_file("mujoco_simulation_mobile_base.xml",
                 "<mujoco model='mobile'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='arm_joint' type='hinge' axis='0 0 1'/>"
                 "      <joint name='steer_left' type='hinge' axis='0 0 1'/>"
                 "      <joint name='drive_left' type='hinge' axis='0 1 0'/>"
                 "      <joint name='passive_joint' type='hinge' axis='1 0 0'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <motor name='arm_joint' joint='arm_joint'/>"
                 "    <motor name='steer_left' joint='steer_left'/>"
                 "    <motor name='drive_left' joint='drive_left'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  const auto info = make_hardware_info(
      {
          make_joint("arm_joint", {"position"}, {"position", "velocity"}),
          make_joint("steer_left", {"position"}, {"position", "velocity"}),
          make_joint("drive_left", {"velocity"}, {"position", "velocity"}),
          make_joint("passive_joint", {}, {"position", "velocity"}),
      },
      {},
      {
          {"mobile_base.type", "custom_joint_group"},
          {"mobile_base.traction_joints", "drive_left"},
          {"mobile_base.steering_joints", "steer_left"},
          {"mobile_base.passive_joints", "passive_joint"},
      });

  mujoco_simulation::MujocoJoints joints;
  std::string error_message;
  ASSERT_TRUE(joints.configure(
      info, *loaded.model,
      [&info](const std::string& key, const std::string& default_value) {
        const auto it = info.hardware_parameters.find(key);
        return it == info.hardware_parameters.end() ? default_value : it->second;
      },
      &error_message))
      << error_message;

  const auto commands = command_names(joints.export_command_interfaces(info.joints));
  const auto states = state_names(joints.export_state_interfaces(info.joints));

  EXPECT_NE(std::find(commands.begin(), commands.end(), "arm_joint/position"), commands.end());
  EXPECT_NE(std::find(commands.begin(), commands.end(), "steer_left/position"), commands.end());
  EXPECT_NE(std::find(commands.begin(), commands.end(), "drive_left/velocity"), commands.end());
  EXPECT_EQ(std::find(commands.begin(), commands.end(), "passive_joint/position"), commands.end());
  EXPECT_NE(std::find(states.begin(), states.end(), "passive_joint/position"), states.end());
  EXPECT_NE(std::find(states.begin(), states.end(), "passive_joint/velocity"), states.end());
  EXPECT_EQ(joints.mobile_base().type, mujoco_simulation::MobileBaseType::CustomJointGroup);
}

TEST(MujocoJointsTest, FailsWhenCommandJointHasNoActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_missing_actuator.xml",
                 "<mujoco model='missing_actuator'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='bad_joint' type='hinge' axis='0 0 1'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  const auto info = make_hardware_info({make_joint("bad_joint", {"position"}, {"position"})});

  mujoco_simulation::MujocoJoints joints;
  std::string error_message;
  EXPECT_FALSE(joints.configure(
      info, *loaded.model,
      [&info](const std::string& key, const std::string& default_value) {
        const auto it = info.hardware_parameters.find(key);
        return it == info.hardware_parameters.end() ? default_value : it->second;
      },
      &error_message));
  EXPECT_NE(error_message.find("MuJoCo actuator not found"), std::string::npos);
}

TEST(MujocoImuTest, ExportsInterfacesAndReadsSensorData) {
  const std::string model_path =
      write_file("mujoco_simulation_imu.xml",
                 "<mujoco model='imu'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <site name='imu_site' pos='0 0 0'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <sensor>"
                 "    <framequat name='imu_quat' objtype='site' objname='imu_site'/>"
                 "    <gyro name='imu_gyro' site='imu_site'/>"
                 "    <accelerometer name='imu_accel' site='imu_site'/>"
                 "  </sensor>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  const std::vector<hardware_interface::ComponentInfo> sensors = {
      make_sensor("imu",
                  {
                      {"mujoco_type", "imu"},
                      {"mujoco_orientation_sensor", "imu_quat"},
                      {"mujoco_gyro_sensor", "imu_gyro"},
                      {"mujoco_accel_sensor", "imu_accel"},
                  },
                  {
                      "orientation.x",
                      "orientation.y",
                      "orientation.z",
                      "orientation.w",
                      "angular_velocity.x",
                      "angular_velocity.y",
                      "angular_velocity.z",
                      "linear_acceleration.x",
                      "linear_acceleration.y",
                      "linear_acceleration.z",
                  })};

  mujoco_simulation::MujocoImus imu;
  std::string error_message;
  ASSERT_TRUE(imu.configure(sensors, *loaded.model, &error_message)) << error_message;

  auto interfaces = imu.export_state_interfaces(sensors);
  const int quat_id = mj_name2id(loaded.model, mjOBJ_SENSOR, "imu_quat");
  const int gyro_id = mj_name2id(loaded.model, mjOBJ_SENSOR, "imu_gyro");
  const int accel_id = mj_name2id(loaded.model, mjOBJ_SENSOR, "imu_accel");
  loaded.data->sensordata[loaded.model->sensor_adr[quat_id]] = 1.0;
  loaded.data->sensordata[loaded.model->sensor_adr[quat_id] + 1] = 0.1;
  loaded.data->sensordata[loaded.model->sensor_adr[quat_id] + 2] = 0.2;
  loaded.data->sensordata[loaded.model->sensor_adr[quat_id] + 3] = 0.3;
  loaded.data->sensordata[loaded.model->sensor_adr[gyro_id]] = 1.1;
  loaded.data->sensordata[loaded.model->sensor_adr[gyro_id] + 1] = 1.2;
  loaded.data->sensordata[loaded.model->sensor_adr[gyro_id] + 2] = 1.3;
  loaded.data->sensordata[loaded.model->sensor_adr[accel_id]] = 2.1;
  loaded.data->sensordata[loaded.model->sensor_adr[accel_id] + 1] = 2.2;
  loaded.data->sensordata[loaded.model->sensor_adr[accel_id] + 2] = 2.3;

  imu.read(*loaded.data);

  EXPECT_DOUBLE_EQ(state_value(interfaces, "imu", "orientation.w"), 1.0);
  EXPECT_DOUBLE_EQ(state_value(interfaces, "imu", "orientation.x"), 0.1);
  EXPECT_DOUBLE_EQ(state_value(interfaces, "imu", "angular_velocity.z"), 1.3);
  EXPECT_DOUBLE_EQ(state_value(interfaces, "imu", "linear_acceleration.y"), 2.2);
}
