#include "mujoco_ros2_bridge/mujoco_ros_bridge.hpp"

#include <chrono>
#include <cmath>

namespace mujoco_ros2_bridge {

MujocoRosBridge::MujocoRosBridge(rclcpp::Node::SharedPtr node,
                                 std::shared_ptr<mujoco_simulation::MuJoCoSimulation> simulation)
    : node_(std::move(node)), simulation_(std::move(simulation)) {}

MujocoRosBridge::~MujocoRosBridge() { stop(); }

bool MujocoRosBridge::initialize(const hardware_interface::HardwareInfo &hardware_info,
                                 const BridgeConfig &config, std::string *error_message) {
  config_ = config;
  if (simulation_ == nullptr || !simulation_->is_initialized()) {
    if (error_message != nullptr) {
      *error_message = "MuJoCoSimulation is not initialized for ROS bridge.";
    }
    return false;
  }

  if (config_.publish_clock) {
    clock_publisher_ =
        node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
  }

  if (config_.enable_sim_services) {
    set_pause_service_ = node_->create_service<mujoco_ros2_bridge_msgs::srv::SetPause>(
        "/mujoco/set_pause",
        [this](const std::shared_ptr<mujoco_ros2_bridge_msgs::srv::SetPause::Request> request,
               std::shared_ptr<mujoco_ros2_bridge_msgs::srv::SetPause::Response> response) {
          response->success = simulation_->set_paused(request->paused);
          response->message = request->paused ? "Simulation paused." : "Simulation resumed.";
        });

    reset_world_service_ = node_->create_service<mujoco_ros2_bridge_msgs::srv::ResetWorld>(
        "/mujoco/reset_world",
        [this](const std::shared_ptr<mujoco_ros2_bridge_msgs::srv::ResetWorld::Request> request,
               std::shared_ptr<mujoco_ros2_bridge_msgs::srv::ResetWorld::Response> response) {
          response->success = simulation_->reset(request->keyframe, &response->message);
          if (response->success && response->message.empty()) {
            response->message = "Simulation reset.";
          }
        });

    step_simulation_service_ = node_->create_service<mujoco_ros2_bridge_msgs::srv::StepSimulation>(
        "/mujoco/step_simulation",
        [this](const std::shared_ptr<mujoco_ros2_bridge_msgs::srv::StepSimulation::Request> request,
               std::shared_ptr<mujoco_ros2_bridge_msgs::srv::StepSimulation::Response> response) {
          response->success = simulation_->step(request->steps, &response->message);
          if (response->success && response->message.empty()) {
            response->message = "Simulation stepped.";
          }
        });
  }

  cameras_ = std::make_unique<mujoco_simulation::MujocoCameras>(node_, simulation_.get());
  if (!cameras_->register_cameras(hardware_info, error_message)) {
    return false;
  }

  lidars_ = std::make_unique<mujoco_simulation::MujocoLidars>(node_, simulation_.get());
  if (!lidars_->register_lidars(hardware_info, error_message)) {
    return false;
  }

  return true;
}

void MujocoRosBridge::start() {
  if (cameras_ != nullptr) {
    cameras_->start();
  }
  if (lidars_ != nullptr) {
    lidars_->start();
  }
  if (config_.publish_clock) {
    clock_timer_ =
        node_->create_wall_timer(std::chrono::milliseconds(20), [this]() { publish_clock_once(); });
  }
}

void MujocoRosBridge::stop() {
  clock_timer_.reset();
  if (cameras_ != nullptr) {
    cameras_->stop();
  }
  if (lidars_ != nullptr) {
    lidars_->stop();
  }
}

void MujocoRosBridge::publish_clock_once() {
  if (clock_publisher_ == nullptr || simulation_ == nullptr) {
    return;
  }

  rosgraph_msgs::msg::Clock clock_msg;
  bool published = false;
  simulation_->with_locked_data([&](const mjModel &, const mjData &data) {
    const auto seconds = static_cast<int32_t>(std::floor(data.time));
    const auto nanoseconds = static_cast<uint32_t>((data.time - seconds) * 1e9);
    clock_msg.clock.sec = seconds;
    clock_msg.clock.nanosec = nanoseconds;
    published = true;
  });
  if (published) {
    clock_publisher_->publish(clock_msg);
  }
}

}  // namespace mujoco_ros2_bridge
