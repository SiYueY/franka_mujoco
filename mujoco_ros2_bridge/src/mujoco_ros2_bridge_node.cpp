#include <chrono>
#include <memory>
#include <thread>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    executor, "controller_manager", "");

  executor->add_node(controller_manager);

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

  rclcpp::shutdown();
  return 0;
}
