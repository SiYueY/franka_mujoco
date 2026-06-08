#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mujoco {
class Simulate;
}  // namespace mujoco

namespace mujoco_simulation {

enum class RenderMode {
  Headless,
  Viewer,
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

  uint64_t step_count() const;
  const mjModel *model() const;
  bool is_initialized() const;
  bool is_running() const;
  const SimulationConfig &config() const;

  void with_locked_data(const std::function<void(const mjModel &, mjData &)> &callback);
  void with_locked_data(const std::function<void(const mjModel &, const mjData &)> &callback) const;
  bool copy_data_to(mjData *dest) const;

 private:
  using SimulateHandle = std::unique_ptr<mujoco::Simulate, void (*)(mujoco::Simulate *)>;

  void physics_loop();
  bool load_model(const std::string &model_path, std::string *error_message);
  bool start_viewer(std::string *error_message);
  int keyframe_id(const std::string &keyframe) const;

  SimulationConfig config_;
  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  mjvCamera camera_;
  mjvOption visual_options_;
  mjvPerturb perturb_;
  SimulateHandle simulate_;
  std::thread viewer_thread_;

  mutable std::mutex mutex_;
  std::thread physics_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool paused_{false};
  std::atomic_uint64_t step_count_{0};
};

RenderMode parse_render_mode(const std::string &value);
const char *to_string(RenderMode mode);

}  // namespace mujoco_simulation
