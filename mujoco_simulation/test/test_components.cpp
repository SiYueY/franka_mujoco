#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>
#include <string>

#include "mujoco_simulation/hardware/hardware_manager.hpp"

namespace {

std::string write_file(const std::string& name, const std::string& contents) {
  const std::string path = "/tmp/" + name;
  std::ofstream out(path);
  out << contents;
  out.close();
  return path;
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
    mj_forward(model, data);
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

TEST(JointTest, ReadsStateAndWritesPositionCommandThroughActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_joint_position.xml",
                 "<mujoco model='joint_position'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='arm_joint' type='hinge' axis='0 0 1'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <motor name='arm_motor' joint='arm_joint'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  ASSERT_TRUE(
      joint.init({"arm_joint", "arm_motor", mujoco_simulation::CommandInterfaceType::Position}))
      << joint.last_error();
  EXPECT_EQ(joint.joint_type(), mujoco_simulation::JointType::Hinge);
  EXPECT_EQ(joint.actuator_type(), mujoco_simulation::ActuatorType::Motor);

  const int joint_id = mj_name2id(loaded.model, mjOBJ_JOINT, "arm_joint");
  loaded.data->qpos[loaded.model->jnt_qposadr[joint_id]] = 1.25;
  loaded.data->qvel[loaded.model->jnt_dofadr[joint_id]] = -0.75;
  loaded.data->qfrc_actuator[loaded.model->jnt_dofadr[joint_id]] = 0.5;
  loaded.data->qfrc_applied[loaded.model->jnt_dofadr[joint_id]] = 0.25;

  mujoco_simulation::JointState state;
  ASSERT_TRUE(joint.read(state)) << joint.last_error();
  EXPECT_DOUBLE_EQ(state.position, 1.25);
  EXPECT_DOUBLE_EQ(state.velocity, -0.75);
  EXPECT_DOUBLE_EQ(state.effort, 0.75);

  ASSERT_TRUE(joint.write({"", 2.5, 0.0, 0.0, 0.0})) << joint.last_error();
  const int actuator_id = mj_name2id(loaded.model, mjOBJ_ACTUATOR, "arm_motor");
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[actuator_id], 2.5);
}

TEST(JointTest, DetectsSlideJointAndPositionActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_joint_slide_position.xml",
                 "<mujoco model='joint_slide_position'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='slider_joint' type='slide' axis='1 0 0'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <position name='slider_position' joint='slider_joint' kp='10'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  ASSERT_TRUE(joint.init(
      {"slider_joint", "slider_position", mujoco_simulation::CommandInterfaceType::Position}))
      << joint.last_error();
  EXPECT_EQ(joint.joint_type(), mujoco_simulation::JointType::Slide);
  EXPECT_EQ(joint.actuator_type(), mujoco_simulation::ActuatorType::Position);
}

TEST(JointTest, WritesEffortWithoutActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_joint_effort.xml",
                 "<mujoco model='joint_effort'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='free_joint' type='hinge' axis='0 0 1'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  ASSERT_TRUE(joint.init({"free_joint", "", mujoco_simulation::CommandInterfaceType::Effort}))
      << joint.last_error();
  EXPECT_EQ(joint.actuator_type(), mujoco_simulation::ActuatorType::Passive);
  ASSERT_TRUE(joint.write({"", 0.0, 0.0, 0.0, 3.2})) << joint.last_error();

  const int joint_id = mj_name2id(loaded.model, mjOBJ_JOINT, "free_joint");
  EXPECT_DOUBLE_EQ(loaded.data->qfrc_applied[loaded.model->jnt_dofadr[joint_id]], 3.2);
}

TEST(JointTest, FailsWhenPositionJointHasNoActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_joint_missing_actuator.xml",
                 "<mujoco model='joint_missing_actuator'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='bad_joint' type='hinge' axis='0 0 1'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  EXPECT_FALSE(joint.init({"bad_joint", "", mujoco_simulation::CommandInterfaceType::Position}));
  EXPECT_NE(joint.last_error().find("passive joint"), std::string::npos);
}

TEST(JointTest, FailsWhenEffortCommandUsesVelocityActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_joint_velocity_actuator.xml",
                 "<mujoco model='joint_velocity_actuator'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='wheel_joint' type='hinge' axis='0 0 1'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <velocity name='wheel_velocity' joint='wheel_joint' kv='1'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  EXPECT_FALSE(joint.init(
      {"wheel_joint", "wheel_velocity", mujoco_simulation::CommandInterfaceType::Effort}));
  EXPECT_NE(joint.last_error().find("position/velocity actuator"), std::string::npos);
}

TEST(JointTest, FailsForBallJoint) {
  const std::string model_path = write_file("mujoco_simulation_ball_joint.xml",
                                            "<mujoco model='ball_joint'>"
                                            "  <worldbody>"
                                            "    <body name='body' pos='0 0 0.1'>"
                                            "      <joint name='ball_joint' type='ball'/>"
                                            "      <geom type='sphere' size='0.05' mass='1'/>"
                                            "    </body>"
                                            "  </worldbody>"
                                            "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  EXPECT_FALSE(joint.init({"ball_joint", "", mujoco_simulation::CommandInterfaceType::None}));
  EXPECT_NE(joint.last_error().find("only supports 1-DoF"), std::string::npos);
}

TEST(JointTest, FailsForFreeJoint) {
  const std::string model_path = write_file("mujoco_simulation_free_joint.xml",
                                            "<mujoco model='free_joint'>"
                                            "  <worldbody>"
                                            "    <body name='body' pos='0 0 0.1'>"
                                            "      <joint name='root_joint' type='free'/>"
                                            "      <geom type='sphere' size='0.05' mass='1'/>"
                                            "    </body>"
                                            "  </worldbody>"
                                            "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  EXPECT_FALSE(joint.init({"root_joint", "", mujoco_simulation::CommandInterfaceType::None}));
  EXPECT_NE(joint.last_error().find("only supports 1-DoF"), std::string::npos);
}

TEST(JointTest, AutoDiscoversJointActuator) {
  const std::string model_path =
      write_file("mujoco_simulation_joint_autodiscovery.xml",
                 "<mujoco model='joint_autodiscovery'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='auto_joint' type='hinge' axis='0 0 1'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <motor name='auto_motor' joint='auto_joint'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Joint joint(loaded.model, loaded.data);
  ASSERT_TRUE(joint.init({"auto_joint", "", mujoco_simulation::CommandInterfaceType::Velocity}))
      << joint.last_error();
  EXPECT_EQ(joint.actuator_type(), mujoco_simulation::ActuatorType::Motor);
}

TEST(LidarTest, BindsBeamNamesWithZeroPaddingAndReadsRanges) {
  const std::string model_path = write_file("mujoco_simulation_lidar.xml",
                                            "<mujoco model='lidar'>"
                                            "  <worldbody>"
                                            "    <body name='body' pos='0 0 0.2'>"
                                            "      <site name='lidar_site' pos='0 0 0'/>"
                                            "      <geom type='sphere' size='0.05' mass='1'/>"
                                            "    </body>"
                                            "  </worldbody>"
                                            "  <sensor>"
                                            "    <rangefinder name='scan-000' site='lidar_site'/>"
                                            "    <rangefinder name='scan-001' site='lidar_site'/>"
                                            "    <rangefinder name='scan-002' site='lidar_site'/>"
                                            "  </sensor>"
                                            "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Lidar lidar(loaded.model, loaded.data);
  ASSERT_TRUE(lidar.init({"front_lidar", "laser_frame", "scan", -1.0, 1.0, 1.0, 0.1, 5.0}))
      << lidar.last_error();

  for (int i = 0; i < 3; ++i) {
    const std::string sensor_name = "scan-00" + std::to_string(i);
    const int sensor_id = mj_name2id(loaded.model, mjOBJ_SENSOR, sensor_name.c_str());
    loaded.data->sensordata[loaded.model->sensor_adr[sensor_id]] = i == 1 ? 6.0 : 1.0 + i;
  }

  mujoco_simulation::LidarState state;
  ASSERT_TRUE(lidar.read(state)) << lidar.last_error();
  ASSERT_EQ(state.laser_scan.ranges.size(), 3U);
  EXPECT_DOUBLE_EQ(state.laser_scan.ranges[0], 1.0);
  EXPECT_DOUBLE_EQ(state.laser_scan.ranges[1], -1.0);
  EXPECT_DOUBLE_EQ(state.laser_scan.ranges[2], 3.0);
}

TEST(LidarTest, FailsWhenBeamIsMissing) {
  const std::string model_path = write_file("mujoco_simulation_lidar_missing_beam.xml",
                                            "<mujoco model='lidar_missing_beam'>"
                                            "  <worldbody>"
                                            "    <body name='body' pos='0 0 0.2'>"
                                            "      <site name='lidar_site' pos='0 0 0'/>"
                                            "      <geom type='sphere' size='0.05' mass='1'/>"
                                            "    </body>"
                                            "  </worldbody>"
                                            "  <sensor>"
                                            "    <rangefinder name='scan-0' site='lidar_site'/>"
                                            "    <rangefinder name='scan-2' site='lidar_site'/>"
                                            "  </sensor>"
                                            "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::Lidar lidar(loaded.model, loaded.data);
  EXPECT_FALSE(lidar.init({"front_lidar", "laser_frame", "scan", -1.0, 1.0, 1.0, 0.1, 5.0}));
  EXPECT_NE(lidar.last_error().find("missing"), std::string::npos);
}

TEST(HardwareManagerTest, RegistersDevicesAndReadsAllStates) {
  const std::string model_path =
      write_file("mujoco_simulation_manager.xml",
                 "<mujoco model='manager'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.2'>"
                 "      <joint name='arm_joint' type='hinge' axis='0 0 1'/>"
                 "      <site name='imu_site' pos='0 0 0'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <motor name='arm_motor' joint='arm_joint'/>"
                 "  </actuator>"
                 "  <sensor>"
                 "    <framequat name='imu_quat' objtype='site' objname='imu_site'/>"
                 "    <gyro name='imu_gyro' site='imu_site'/>"
                 "    <accelerometer name='imu_accel' site='imu_site'/>"
                 "    <rangefinder name='scan-0' site='imu_site'/>"
                 "  </sensor>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::HardwareManager manager(loaded.model, loaded.data);

  ASSERT_TRUE(manager.register_joint(
      {"arm_joint", "arm_motor", mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_imu({"imu", "imu_quat", "imu_gyro", "imu_accel"}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_lidar({"lidar", "laser_frame", "scan", 0.0, 0.0, 1.0, 0.1, 10.0}))
      << manager.last_error();

  loaded.data->qpos[loaded.model->jnt_qposadr[mj_name2id(loaded.model, mjOBJ_JOINT, "arm_joint")]] =
      0.5;
  loaded.data->qvel[loaded.model->jnt_dofadr[mj_name2id(loaded.model, mjOBJ_JOINT, "arm_joint")]] =
      -1.0;
  loaded.data
      ->sensordata[loaded.model->sensor_adr[mj_name2id(loaded.model, mjOBJ_SENSOR, "scan-0")]] =
      2.5;

  ASSERT_TRUE(manager.write_joint("arm_joint", {"", 0.0, 4.2, 0.0, 0.0})) << manager.last_error();
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[mj_name2id(loaded.model, mjOBJ_ACTUATOR, "arm_motor")], 4.2);

  auto joint_states = manager.read_joint_states();
  auto lidar_states = manager.read_lidar_states();
  ASSERT_EQ(joint_states.size(), 1U);
  ASSERT_EQ(lidar_states.size(), 1U);
  EXPECT_DOUBLE_EQ(joint_states.at("arm_joint").position, 0.5);
  EXPECT_DOUBLE_EQ(lidar_states.at("lidar").laser_scan.ranges.at(0), 2.5);

  ASSERT_TRUE(manager.reset_all()) << manager.last_error();
  ASSERT_TRUE(manager.unregister_lidar("lidar")) << manager.last_error();

  mujoco_simulation::LidarState lidar_state;
  EXPECT_FALSE(manager.read_lidar("lidar", lidar_state));
  EXPECT_NE(manager.last_error().find("not found"), std::string::npos);
}

TEST(MobileBaseTest, DifferentialBaseMapsTwistToWheelVelocities) {
  const std::string model_path =
      write_file("mujoco_simulation_diff_base.xml",
                 "<mujoco model='diff_base'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='left_wheel_joint' type='hinge' axis='0 1 0'/>"
                 "      <joint name='right_wheel_joint' type='hinge' axis='0 1 0'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <motor name='left_motor' joint='left_wheel_joint'/>"
                 "    <motor name='right_motor' joint='right_wheel_joint'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::HardwareManager manager(loaded.model, loaded.data);
  ASSERT_TRUE(manager.register_joint(
      {"left_wheel_joint", "left_motor", mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_joint(
      {"right_wheel_joint", "right_motor", mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_mobile_base({"base",
                                            mujoco_simulation::MobileBaseType::Differential,
                                            "base_link",
                                            "odom",
                                            {"left_wheel_joint", "right_wheel_joint"},
                                            {},
                                            0.2,
                                            0.6,
                                            0.0}))
      << manager.last_error();

  ASSERT_TRUE(manager.write_mobile_base("base", {{1.0, 0.0, 0.0}, {0.0, 0.0, 0.5}}))
      << manager.last_error();
  const int left_actuator = mj_name2id(loaded.model, mjOBJ_ACTUATOR, "left_motor");
  const int right_actuator = mj_name2id(loaded.model, mjOBJ_ACTUATOR, "right_motor");
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[left_actuator], 4.25);
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[right_actuator], 5.75);

  const int left_joint = mj_name2id(loaded.model, mjOBJ_JOINT, "left_wheel_joint");
  const int right_joint = mj_name2id(loaded.model, mjOBJ_JOINT, "right_wheel_joint");
  loaded.data->qvel[loaded.model->jnt_dofadr[left_joint]] = 4.25;
  loaded.data->qvel[loaded.model->jnt_dofadr[right_joint]] = 5.75;

  mujoco_simulation::MobileBaseState state;
  ASSERT_TRUE(manager.read_mobile_base("base", state)) << manager.last_error();
  EXPECT_DOUBLE_EQ(state.linear[0], 1.0);
  EXPECT_DOUBLE_EQ(state.linear[1], 0.0);
  EXPECT_DOUBLE_EQ(state.angular[2], 0.5);
}

TEST(MobileBaseTest, OmnidirectionalBaseMapsTwistToWheelVelocities) {
  const std::string model_path =
      write_file("mujoco_simulation_omni_base.xml",
                 "<mujoco model='omni_base'>"
                 "  <worldbody>"
                 "    <body name='body' pos='0 0 0.1'>"
                 "      <joint name='front_left_joint' type='hinge' axis='0 1 0'/>"
                 "      <joint name='front_right_joint' type='hinge' axis='0 1 0'/>"
                 "      <joint name='rear_left_joint' type='hinge' axis='0 1 0'/>"
                 "      <joint name='rear_right_joint' type='hinge' axis='0 1 0'/>"
                 "      <geom type='sphere' size='0.05' mass='1'/>"
                 "    </body>"
                 "  </worldbody>"
                 "  <actuator>"
                 "    <motor name='front_left_motor' joint='front_left_joint'/>"
                 "    <motor name='front_right_motor' joint='front_right_joint'/>"
                 "    <motor name='rear_left_motor' joint='rear_left_joint'/>"
                 "    <motor name='rear_right_motor' joint='rear_right_joint'/>"
                 "  </actuator>"
                 "</mujoco>");

  LoadedModel loaded(model_path);
  mujoco_simulation::HardwareManager manager(loaded.model, loaded.data);
  ASSERT_TRUE(manager.register_joint(
      {"front_left_joint", "front_left_motor", mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_joint({"front_right_joint", "front_right_motor",
                                      mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_joint(
      {"rear_left_joint", "rear_left_motor", mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_joint(
      {"rear_right_joint", "rear_right_motor", mujoco_simulation::CommandInterfaceType::Velocity}))
      << manager.last_error();
  ASSERT_TRUE(manager.register_mobile_base(
      {"omni",
       mujoco_simulation::MobileBaseType::Omnidirectional,
       "base_link",
       "odom",
       {"front_left_joint", "front_right_joint", "rear_left_joint", "rear_right_joint"},
       {},
       0.1,
       0.4,
       0.3}))
      << manager.last_error();

  ASSERT_TRUE(manager.write_mobile_base("omni", {{1.0, 0.5, 0.0}, {0.0, 0.0, 0.2}}))
      << manager.last_error();
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[mj_name2id(loaded.model, mjOBJ_ACTUATOR, "front_left_motor")],
                   3.6);
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[mj_name2id(loaded.model, mjOBJ_ACTUATOR, "front_right_motor")],
                   16.4);
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[mj_name2id(loaded.model, mjOBJ_ACTUATOR, "rear_left_motor")],
                   13.6);
  EXPECT_DOUBLE_EQ(loaded.data->ctrl[mj_name2id(loaded.model, mjOBJ_ACTUATOR, "rear_right_motor")],
                   6.4);

  const int fl_joint = mj_name2id(loaded.model, mjOBJ_JOINT, "front_left_joint");
  const int fr_joint = mj_name2id(loaded.model, mjOBJ_JOINT, "front_right_joint");
  const int rl_joint = mj_name2id(loaded.model, mjOBJ_JOINT, "rear_left_joint");
  const int rr_joint = mj_name2id(loaded.model, mjOBJ_JOINT, "rear_right_joint");
  loaded.data->qvel[loaded.model->jnt_dofadr[fl_joint]] = 3.6;
  loaded.data->qvel[loaded.model->jnt_dofadr[fr_joint]] = 16.4;
  loaded.data->qvel[loaded.model->jnt_dofadr[rl_joint]] = 13.6;
  loaded.data->qvel[loaded.model->jnt_dofadr[rr_joint]] = 6.4;

  mujoco_simulation::MobileBaseState state;
  ASSERT_TRUE(manager.read_mobile_base("omni", state)) << manager.last_error();
  EXPECT_NEAR(state.linear[0], 1.0, 1e-9);
  EXPECT_NEAR(state.linear[1], 0.5, 1e-9);
  EXPECT_NEAR(state.angular[2], 0.2, 1e-9);
}
