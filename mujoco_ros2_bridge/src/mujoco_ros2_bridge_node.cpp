#include <chrono>
#include <controller_manager/controller_manager.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <thread>

#include "mujoco_ros2_bridge/mujoco_context.hpp"
#include "mujoco_ros2_bridge/mujoco_ros_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto simulation = std::make_shared<mujoco_simulation::MuJoCoSimulation>();
  mujoco_ros2_bridge::MujocoContext::set_simulation(simulation);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    executor, "controller_manager", "");

  auto bridge_node = std::make_shared<rclcpp::Node>("mujoco_ros2_bridge");
  auto bridge = std::make_shared<mujoco_ros2_bridge::MujocoRosBridge>(bridge_node, simulation);

  const auto hardware_info = mujoco_ros2_bridge::MujocoContext::hardware_info();
  if (!hardware_info.has_value()) {
    RCLCPP_ERROR(bridge_node->get_logger(), "HardwareInfo is not available from MujocoContext.");
    rclcpp::shutdown();
    return 1;
  }

  mujoco_ros2_bridge::BridgeConfig bridge_config;
  const auto publish_clock_it = hardware_info->hardware_parameters.find("publish_clock");
  if (publish_clock_it != hardware_info->hardware_parameters.end()) {
    bridge_config.publish_clock = publish_clock_it->second != "false";
  }
  const auto service_it = hardware_info->hardware_parameters.find("enable_sim_services");
  if (service_it != hardware_info->hardware_parameters.end()) {
    bridge_config.enable_sim_services = service_it->second != "false";
  }

  std::string error_message;
  if (!bridge->initialize(*hardware_info, bridge_config, &error_message)) {
    RCLCPP_ERROR(bridge_node->get_logger(), "%s", error_message.c_str());
    rclcpp::shutdown();
    return 1;
  }

  executor->add_node(controller_manager);
  executor->add_node(bridge_node);
  bridge->start();

  std::thread control_thread([controller_manager]() {
    const int update_rate = controller_manager->get_update_rate();
    const auto period = std::chrono::nanoseconds(1'000'000'000 / update_rate);
    auto next_iteration = std::chrono::steady_clock::now();
    rclcpp::Time previous_time = controller_manager->now();

    const int thread_priority = controller_manager->get_parameter_or<int>("thread_priority", 50);
    if (!realtime_tools::configure_sched_fifo(thread_priority)) {
      RCLCPP_WARN(
        controller_manager->get_logger(),
        "Could not enable FIFO scheduling for controller_manager.");
    }

    while (rclcpp::ok()) {
      const rclcpp::Time current_time = controller_manager->now();
      const rclcpp::Duration measured_period = current_time - previous_time;
      previous_time = current_time;

      controller_manager->read(current_time, measured_period);
      controller_manager->update(current_time, measured_period);
      controller_manager->write(current_time, measured_period);

      next_iteration += period;
      std::this_thread::sleep_until(next_iteration);
    }
  });

  executor->spin();
  if (control_thread.joinable()) {
    control_thread.join();
  }

  bridge->stop();
  rclcpp::shutdown();
  return 0;
}
