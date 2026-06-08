#include <algorithm>
#include <cmath>
#include <cstring>

#include "mujoco_simulation/mujoco_camera.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace mujoco_simulation {
namespace {

std::string sensor_parameter(const hardware_interface::ComponentInfo &sensor,
                             const std::string &key, const std::string &default_value = "") {
  const auto it = sensor.parameters.find(key);
  return it == sensor.parameters.end() ? default_value : it->second;
}

}  // namespace

MujocoCameras::MujocoCameras(rclcpp::Node::SharedPtr node, MuJoCoSimulation *simulation)
    : node_(std::move(node)), simulation_(simulation) {}

MujocoCameras::~MujocoCameras() { stop(); }

bool MujocoCameras::register_cameras(const hardware_interface::HardwareInfo &hardware_info,
                                     std::string *error_message) {
  cameras_.clear();
  const mjModel *model = simulation_ != nullptr ? simulation_->model() : nullptr;
  if (model == nullptr) {
    if (error_message != nullptr) {
      *error_message = "MuJoCo model is not loaded for camera registration.";
    }
    return false;
  }

  for (const auto &sensor : hardware_info.sensors) {
    if (sensor_parameter(sensor, "mujoco_type") != "camera") {
      continue;
    }

    const std::string mujoco_camera_name = sensor_parameter(sensor, "mujoco_camera_name");
    if (mujoco_camera_name.empty()) {
      if (error_message != nullptr) {
        *error_message = "Camera sensor '" + sensor.name + "' requires mujoco_camera_name.";
      }
      return false;
    }

    const int camera_id = mj_name2id(model, mjOBJ_CAMERA, mujoco_camera_name.c_str());
    if (camera_id < 0) {
      if (error_message != nullptr) {
        *error_message = "MuJoCo camera not found: " + mujoco_camera_name;
      }
      return false;
    }

    CameraData binding;
    binding.sensor_name = sensor.name;
    binding.mujoco_camera_name = mujoco_camera_name;
    binding.frame_name = sensor_parameter(sensor, "frame_name", sensor.name);
    binding.image_topic = sensor_parameter(sensor, "image_topic", "/" + sensor.name + "/image_raw");
    binding.camera_info_topic =
        sensor_parameter(sensor, "camera_info_topic", "/" + sensor.name + "/camera_info");
    binding.depth_topic =
        sensor_parameter(sensor, "depth_topic", "/" + sensor.name + "/depth/image_raw");
    binding.publish_rate = std::stod(sensor_parameter(sensor, "publish_rate", "5.0"));
    binding.camera_id = camera_id;
    binding.width = static_cast<uint32_t>(model->cam_resolution[2 * camera_id]);
    binding.height = static_cast<uint32_t>(model->cam_resolution[2 * camera_id + 1]);
    binding.viewport = {0, 0, static_cast<int>(binding.width), static_cast<int>(binding.height)};
    binding.mjv_camera.type = mjCAMERA_FIXED;
    binding.mjv_camera.fixedcamid = camera_id;

    const auto image_size = binding.width * binding.height * 3U;
    binding.image_buffer.resize(image_size);
    binding.depth_buffer.resize(binding.width * binding.height);
    binding.depth_buffer_flipped.resize(binding.width * binding.height);

    binding.image_msg.header.frame_id = binding.frame_name;
    binding.image_msg.width = binding.width;
    binding.image_msg.height = binding.height;
    binding.image_msg.step = binding.width * 3U;
    binding.image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    binding.image_msg.data.resize(image_size);

    binding.depth_msg.header.frame_id = binding.frame_name;
    binding.depth_msg.width = binding.width;
    binding.depth_msg.height = binding.height;
    binding.depth_msg.step = binding.width * sizeof(float);
    binding.depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    binding.depth_msg.data.resize(binding.width * binding.height * sizeof(float));

    binding.camera_info_msg.header.frame_id = binding.frame_name;
    binding.camera_info_msg.width = binding.width;
    binding.camera_info_msg.height = binding.height;
    binding.camera_info_msg.distortion_model = "plumb_bob";
    binding.camera_info_msg.k.fill(0.0);
    binding.camera_info_msg.r.fill(0.0);
    binding.camera_info_msg.p.fill(0.0);
    binding.camera_info_msg.d.resize(5, 0.0);

    const double fovy = model->cam_fovy[camera_id];
    const double focal_scaling =
        (1.0 / std::tan((fovy * M_PI / 180.0) / 2.0)) * static_cast<double>(binding.height) / 2.0;
    binding.camera_info_msg.k[0] = binding.camera_info_msg.p[0] = focal_scaling;
    binding.camera_info_msg.k[2] = binding.camera_info_msg.p[2] =
        static_cast<double>(binding.width) / 2.0;
    binding.camera_info_msg.k[4] = binding.camera_info_msg.p[5] = focal_scaling;
    binding.camera_info_msg.k[5] = binding.camera_info_msg.p[6] =
        static_cast<double>(binding.height) / 2.0;
    binding.camera_info_msg.k[8] = binding.camera_info_msg.p[10] = 1.0;

    binding.image_publisher =
        node_->create_publisher<sensor_msgs::msg::Image>(binding.image_topic, rclcpp::QoS(1));
    binding.depth_publisher =
        node_->create_publisher<sensor_msgs::msg::Image>(binding.depth_topic, rclcpp::QoS(1));
    binding.camera_info_publisher = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        binding.camera_info_topic, rclcpp::QoS(1));
    cameras_.push_back(std::move(binding));
  }

  return true;
}

void MujocoCameras::start() {
  if (cameras_.empty() || publish_images_.exchange(true)) {
    return;
  }
  render_thread_ = std::thread(&MujocoCameras::update_loop, this);
}

void MujocoCameras::stop() {
  publish_images_.store(false);
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
}

bool MujocoCameras::init_egl(std::string *error_message) {
#if MUJOCO_ROS2_BRIDGE_HAS_EGL
  egl_display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (egl_display_ == EGL_NO_DISPLAY) {
    if (error_message != nullptr) {
      *error_message = "Failed to get EGL display.";
    }
    return false;
  }
  EGLint major = 0;
  EGLint minor = 0;
  if (!eglInitialize(egl_display_, &major, &minor)) {
    if (error_message != nullptr) {
      *error_message = "Failed to initialize EGL.";
    }
    return false;
  }
  const EGLint config_attribs[] = {EGL_SURFACE_TYPE,
                                   EGL_PBUFFER_BIT,
                                   EGL_RED_SIZE,
                                   8,
                                   EGL_GREEN_SIZE,
                                   8,
                                   EGL_BLUE_SIZE,
                                   8,
                                   EGL_ALPHA_SIZE,
                                   8,
                                   EGL_DEPTH_SIZE,
                                   24,
                                   EGL_RENDERABLE_TYPE,
                                   EGL_OPENGL_BIT,
                                   EGL_NONE};
  EGLConfig config;
  EGLint count = 0;
  if (!eglChooseConfig(egl_display_, config_attribs, &config, 1, &count) || count == 0) {
    if (error_message != nullptr) {
      *error_message = "Failed to choose EGL config.";
    }
    cleanup_egl();
    return false;
  }
  if (!eglBindAPI(EGL_OPENGL_API)) {
    if (error_message != nullptr) {
      *error_message = "Failed to bind EGL OpenGL API.";
    }
    cleanup_egl();
    return false;
  }
  egl_context_ = eglCreateContext(egl_display_, config, EGL_NO_CONTEXT, nullptr);
  if (egl_context_ == EGL_NO_CONTEXT) {
    if (error_message != nullptr) {
      *error_message = "Failed to create EGL context.";
    }
    cleanup_egl();
    return false;
  }
  const EGLint pbuffer_attribs[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
  egl_surface_ = eglCreatePbufferSurface(egl_display_, config, pbuffer_attribs);
  if (egl_surface_ == EGL_NO_SURFACE) {
    if (error_message != nullptr) {
      *error_message = "Failed to create EGL surface.";
    }
    cleanup_egl();
    return false;
  }
  if (!eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_)) {
    if (error_message != nullptr) {
      *error_message = "Failed to activate EGL context.";
    }
    cleanup_egl();
    return false;
  }
  return true;
#else
  if (error_message != nullptr) {
    *error_message = "EGL headers are not available.";
  }
  return false;
#endif
}

void MujocoCameras::cleanup_egl() {
#if MUJOCO_ROS2_BRIDGE_HAS_EGL
  if (egl_display_ != EGL_NO_DISPLAY) {
    eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (egl_surface_ != EGL_NO_SURFACE) {
      eglDestroySurface(egl_display_, egl_surface_);
      egl_surface_ = EGL_NO_SURFACE;
    }
    if (egl_context_ != EGL_NO_CONTEXT) {
      eglDestroyContext(egl_display_, egl_context_);
      egl_context_ = EGL_NO_CONTEXT;
    }
    eglTerminate(egl_display_);
    egl_display_ = EGL_NO_DISPLAY;
  }
#endif
}

void MujocoCameras::update_loop() {
  GLFWwindow *window = nullptr;
  if (glfwInit()) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window = glfwCreateWindow(1, 1, "", nullptr, nullptr);
    if (window != nullptr) {
      glfwMakeContextCurrent(window);
      use_egl_ = false;
    }
  }
  if (window == nullptr) {
    std::string error_message;
    if (!init_egl(&error_message)) {
      RCLCPP_ERROR(node_->get_logger(), "Camera rendering initialization failed: %s",
                   error_message.c_str());
      publish_images_.store(false);
      if (glfwInit()) {
        glfwTerminate();
      }
      return;
    }
    use_egl_ = true;
  }

  mjv_defaultOption(&scene_options_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&render_context_);
  scene_options_.flags[mjVIS_RANGEFINDER] = 0;
  for (int i = 0; i < mjNGROUP; ++i) {
    scene_options_.sitegroup[i] = 0;
  }

  simulation_->with_locked_data([this](const mjModel &model, mjData &) {
    render_data_ = mj_makeData(&model);
    mjv_makeScene(&model, &scene_, 2000);
    mjr_makeContext(&model, &render_context_, mjFONTSCALE_150);
    int max_width = 1;
    int max_height = 1;
    for (const auto &camera : cameras_) {
      max_width = std::max(max_width, static_cast<int>(camera.width));
      max_height = std::max(max_height, static_cast<int>(camera.height));
    }
    mjr_resizeOffscreen(max_width, max_height, &render_context_);
  });

  double max_publish_rate = 1.0;
  for (const auto &camera : cameras_) {
    max_publish_rate = std::max(max_publish_rate, camera.publish_rate);
  }
  rclcpp::Rate rate(max_publish_rate);
  while (rclcpp::ok() && publish_images_.load()) {
    update_once();
    rate.sleep();
  }

  simulation_->with_locked_data([this](const mjModel &, mjData &) {
    if (render_data_ != nullptr) {
      mj_deleteData(render_data_);
      render_data_ = nullptr;
    }
  });
  mjv_freeScene(&scene_);
  mjr_freeContext(&render_context_);
  if (use_egl_) {
    cleanup_egl();
  } else if (window != nullptr) {
    glfwDestroyWindow(window);
    glfwTerminate();
  }
}

void MujocoCameras::update_once() {
  if (render_data_ == nullptr) {
    return;
  }

  simulation_->with_locked_data(
      [this](const mjModel &model, mjData &data) { mjv_copyData(render_data_, &model, &data); });

  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  const mjModel *model = simulation_->model();
  if (model == nullptr) {
    return;
  }

  for (auto &camera : cameras_) {
    mjv_updateScene(model, render_data_, &scene_options_, nullptr, &camera.mjv_camera, mjCAT_ALL,
                    &scene_);
    mjr_render(camera.viewport, &scene_, &render_context_);
    mjr_readPixels(camera.image_buffer.data(), camera.depth_buffer.data(), camera.viewport,
                   &render_context_);
  }

  const float near = static_cast<float>(model->vis.map.znear * model->stat.extent);
  const float far = static_cast<float>(model->vis.map.zfar * model->stat.extent);
  const float depth_scale = 1.0f - near / far;
  const auto now = node_->now();
  for (auto &camera : cameras_) {
    for (uint32_t h = 0; h < camera.height; ++h) {
      for (uint32_t w = 0; w < camera.width; ++w) {
        const auto idx = h * camera.width + w;
        const auto flipped_idx = (camera.height - 1U - h) * camera.width + w;
        camera.depth_buffer[idx] = near / (1.0f - camera.depth_buffer[idx] * depth_scale);
        camera.depth_buffer_flipped[flipped_idx] = camera.depth_buffer[idx];
      }
    }
    std::memcpy(camera.depth_msg.data.data(), camera.depth_buffer_flipped.data(),
                camera.depth_msg.data.size());

    const auto row_size = camera.width * 3U;
    for (uint32_t h = 0; h < camera.height; ++h) {
      const auto src_idx = h * row_size;
      const auto dst_idx = (camera.height - 1U - h) * row_size;
      std::memcpy(&camera.image_msg.data[dst_idx], &camera.image_buffer[src_idx], row_size);
    }

    camera.image_msg.header.stamp = now;
    camera.depth_msg.header.stamp = now;
    camera.camera_info_msg.header.stamp = now;
    camera.image_publisher->publish(camera.image_msg);
    camera.depth_publisher->publish(camera.depth_msg);
    camera.camera_info_publisher->publish(camera.camera_info_msg);
  }
}

}  // namespace mujoco_simulation
