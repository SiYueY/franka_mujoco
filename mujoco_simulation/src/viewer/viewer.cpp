#include "mujoco_simulation/viewer/viewer.hpp"

#include <exception>
#include <mutex>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {
namespace {

void delete_simulate(mujoco::Simulate* simulate) { delete simulate; }

}  // namespace

Viewer::Viewer() : simulate_(nullptr, delete_simulate) {}

Viewer::~Viewer() { stop(); }

bool Viewer::initialize(std::string* error_message) {
  if (simulate_ != nullptr) {
    if (error_message != nullptr) {
      *error_message = "Viewer is already initialized.";
    }
    return false;
  }

  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);
    simulate_.reset(new mujoco::Simulate(std::make_unique<mujoco::GlfwAdapter>(), &camera_,
                                         &visual_options_, &perturb_, false));
  } catch (const std::exception& exc) {
    if (error_message != nullptr) {
      *error_message = std::string("Failed to initialize MuJoCo viewer: ") + exc.what();
    }
    return false;
  }

  return true;
}

bool Viewer::load(mjModel* model, mjData* data, const std::string& displayed_filename,
                  std::string* error_message) {
  if (simulate_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Viewer must be initialized before loading a model.";
    }
    return false;
  }
  if (model == nullptr || data == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Viewer requires non-null MuJoCo model and data.";
    }
    return false;
  }

  pending_model_ = model;
  pending_data_ = data;
  pending_displayed_filename_ = displayed_filename;
  has_pending_load_ = true;

  if (is_running()) {
    simulate_->Load(pending_model_, pending_data_, pending_displayed_filename_.c_str());
    clear_pending_load();
  }

  return true;
}

bool Viewer::start(std::string* error_message) {
  if (simulate_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Viewer must be initialized before starting.";
    }
    return false;
  }
  if (render_thread_.joinable()) {
    return true;
  }

  try {
    simulate_->exitrequest.store(false);
    render_thread_ = std::thread([this]() { simulate_->RenderLoop(); });
    if (has_pending_load_) {
      simulate_->Load(pending_model_, pending_data_, pending_displayed_filename_.c_str());
      clear_pending_load();
    }
  } catch (const std::exception& exc) {
    if (render_thread_.joinable()) {
      simulate_->exitrequest.store(true);
      render_thread_.join();
    }
    if (error_message != nullptr) {
      *error_message = std::string("Failed to start MuJoCo viewer: ") + exc.what();
    }
    return false;
  }

  return true;
}

void Viewer::stop() {
  if (simulate_ != nullptr) {
    simulate_->exitrequest.store(true);
  }
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
}

bool Viewer::sync(bool state_only, std::string* error_message) {
  if (simulate_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Viewer is not initialized.";
    }
    return false;
  }

  try {
    std::unique_lock<std::recursive_mutex> lock(simulate_->mtx);
    if (simulate_->exitrequest.load()) {
      if (error_message != nullptr) {
        *error_message = "Viewer is stopping.";
      }
      return false;
    }
    simulate_->Sync(state_only);
  } catch (const std::exception& exc) {
    if (error_message != nullptr) {
      *error_message = std::string("Failed to sync MuJoCo viewer: ") + exc.what();
    }
    return false;
  }

  return true;
}

bool Viewer::is_initialized() const { return simulate_ != nullptr; }

bool Viewer::is_running() const { return render_thread_.joinable(); }

mjvScene* Viewer::scene() {
  if (simulate_ == nullptr) {
    return nullptr;
  }
  return &simulate_->scn;
}

mjrContext* Viewer::render_context() {
  if (simulate_ == nullptr || simulate_->platform_ui == nullptr) {
    return nullptr;
  }
  return &simulate_->platform_ui->mjr_context();
}

mjvCamera* Viewer::camera() { return is_initialized() ? &camera_ : nullptr; }

mjvOption* Viewer::visual_options() { return is_initialized() ? &visual_options_ : nullptr; }

mjvPerturb* Viewer::perturb() { return is_initialized() ? &perturb_ : nullptr; }

void Viewer::clear_pending_load() {
  pending_model_ = nullptr;
  pending_data_ = nullptr;
  pending_displayed_filename_.clear();
  has_pending_load_ = false;
}

}  // namespace mujoco_simulation
