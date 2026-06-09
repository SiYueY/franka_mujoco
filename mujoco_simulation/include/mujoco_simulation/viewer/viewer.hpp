#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <thread>

namespace mujoco {
class Simulate;
}  // namespace mujoco

namespace mujoco_simulation {

class Viewer {
 public:
  Viewer();
  ~Viewer();

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;
  Viewer(Viewer&&) = delete;
  Viewer& operator=(Viewer&&) = delete;

  bool initialize(std::string* error_message = nullptr);
  bool load(mjModel* model, mjData* data, const std::string& displayed_filename,
            std::string* error_message = nullptr);
  bool start(std::string* error_message = nullptr);
  void stop();

  bool sync(bool state_only = false, std::string* error_message = nullptr);
  bool is_initialized() const;
  bool is_running() const;

  mjvScene* scene();
  mjrContext* render_context();
  mjvCamera* camera();
  mjvOption* visual_options();
  mjvPerturb* perturb();

 private:
  using SimulateHandle = std::unique_ptr<mujoco::Simulate, void (*)(mujoco::Simulate*)>;

  void clear_pending_load();

  mjvCamera camera_{};
  mjvOption visual_options_{};
  mjvPerturb perturb_{};
  SimulateHandle simulate_;
  std::thread render_thread_;
  mjModel* pending_model_{nullptr};
  mjData* pending_data_{nullptr};
  std::string pending_displayed_filename_;
  bool has_pending_load_{false};
};

}  // namespace mujoco_simulation
