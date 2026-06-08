#include "mujoco_simulation/mujoco_simulation.hpp"

#include <chrono>
#include <stdexcept>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {
namespace {
constexpr int kLoadErrorLength = 1024;

void delete_simulate(mujoco::Simulate *simulate) { delete simulate; }
}  // namespace

MuJoCoSimulation::MuJoCoSimulation() : simulate_(nullptr, delete_simulate) {}

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
  if (!running_.exchange(false)) {
    return;
  }
  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }
  if (simulate_ != nullptr) {
    simulate_->exitrequest.store(true);
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
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

  std::unique_lock<std::recursive_mutex> viewer_lock;
  if (simulate_ != nullptr) {
    viewer_lock = std::unique_lock<std::recursive_mutex>(simulate_->mtx);
  }

  for (uint32_t i = 0; i < steps; ++i) {
    mj_step(model_, data_);
    ++step_count_;
  }
  if (simulate_ != nullptr) {
    simulate_->Sync();
  }
  return true;
}

uint64_t MuJoCoSimulation::step_count() const { return step_count_.load(); }

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
  mj_forward(model_, data_);
  return true;
}

bool MuJoCoSimulation::start_viewer(std::string *error_message) {
  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);
    simulate_.reset(new mujoco::Simulate(std::make_unique<mujoco::GlfwAdapter>(), &camera_,
                                         &visual_options_, &perturb_, false));

    simulate_->Load(model_, data_, config_.model_path.c_str());
    viewer_thread_ = std::thread([this]() { simulate_->RenderLoop(); });
  } catch (const std::exception &exc) {
    if (error_message != nullptr) {
      *error_message = std::string("Failed to start MuJoCo viewer: ") + exc.what();
    }
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
