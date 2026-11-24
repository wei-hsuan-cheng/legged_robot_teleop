// Run until interrupted; zeroing on shutdown is not a hardware E-stop.
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "legged_robot_teleop/base_twist_teleop.hpp"

namespace
{
std::atomic<bool> g_interrupted{false};
void interrupt(int) {g_interrupted = true;}
}  // namespace

int main(int argc, char ** argv)
{
  // Own the signal so one last zero command can be published before the context goes down.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  int code = 0;
  {
    std::shared_ptr<legged_robot_teleop::BaseTwistTeleop> node;
    try {
      node = std::make_shared<legged_robot_teleop::BaseTwistTeleop>();
    } catch (const std::exception & error) {
      RCLCPP_FATAL(rclcpp::get_logger("base_twist_teleop"), "%s", error.what());
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !g_interrupted) {
      executor.spin_once(std::chrono::milliseconds(10));
    }
    if (rclcpp::ok()) {
      node->stopAndPublish("shutdown");
    }
  }
  rclcpp::shutdown();
  return code;
}
