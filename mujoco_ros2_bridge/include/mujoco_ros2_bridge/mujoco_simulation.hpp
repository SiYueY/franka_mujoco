#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_bridge_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_bridge_msgs/srv/set_pause.hpp>
#include <mujoco_ros2_bridge_msgs/srv/step_simulation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace mujoco
{
class Simulate;
}  // namespace mujoco

namespace mujoco_ros2_bridge
{

enum class RenderMode
{
  Headless,
  Viewer,
};

struct SimulationConfig
{
  std::string model_path;
  RenderMode render_mode = RenderMode::Headless;
  double sim_speed_factor = 1.0;
  bool publish_clock = true;
  std::string initial_keyframe;
};

class MuJoCoSimulation
{
public:
  explicit MuJoCoSimulation(rclcpp::Node::SharedPtr node);
  ~MuJoCoSimulation();

  MuJoCoSimulation(const MuJoCoSimulation &) = delete;
  MuJoCoSimulation & operator=(const MuJoCoSimulation &) = delete;

  bool initialize(const SimulationConfig & config, std::string * error_message = nullptr);
  void start();
  void stop();

  bool set_paused(bool paused);
  bool paused() const;
  bool reset(const std::string & keyframe, std::string * error_message = nullptr);
  bool step(uint32_t steps, std::string * error_message = nullptr);

  uint64_t step_count() const;
  const mjModel * model() const;

  void with_locked_data(const std::function<void(const mjModel &, mjData &)> & callback);
  void with_locked_data(const std::function<void(const mjModel &, const mjData &)> & callback) const;

private:
  void physics_loop();
  void publish_clock();
  bool load_model(const std::string & model_path, std::string * error_message);
  bool start_viewer(std::string * error_message);
  int keyframe_id(const std::string & keyframe) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::SetPause>::SharedPtr set_pause_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::ResetWorld>::SharedPtr reset_world_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::StepSimulation>::SharedPtr step_simulation_service_;

  SimulationConfig config_;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  mjvCamera camera_;
  mjvOption visual_options_;
  mjvPerturb perturb_;
  std::unique_ptr<mujoco::Simulate> simulate_;
  std::thread viewer_thread_;

  mutable std::mutex mutex_;
  std::thread physics_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool paused_{false};
  std::atomic_uint64_t step_count_{0};
};

RenderMode parse_render_mode(const std::string & value);
const char * to_string(RenderMode mode);

}  // namespace mujoco_ros2_bridge
