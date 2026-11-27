// Publish the same normalized base command message as the MPC GUI.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/vector3.hpp>
#include <ocs2_msgs/msg/walking_velocity_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "legged_robot_teleop/control.hpp"

namespace legged_robot_teleop
{

/// Convert Joy messages to bounded base commands on a steady timer.
class BaseTwistTeleop : public rclcpp::Node
{
public:
  explicit BaseTwistTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const TeleopControl & control() const {return *control_;}

  /// Clear permission to move and publish one zero-velocity command.
  void stopAndPublish(const std::string & reason);

private:
  void receive(const sensor_msgs::msg::Joy::ConstSharedPtr & message);
  void receiveMotionState(const std_msgs::msg::String::ConstSharedPtr & message);
  void receivePostureButton(const sensor_msgs::msg::Joy & message);
  void receiveGaitButtons(const sensor_msgs::msg::Joy & message);
  void publishGait(const std::string & gait);
  void publish();
  static double monotonic();

  std::unique_ptr<TeleopControl> control_;
  int posture_button_;
  std::optional<bool> posture_button_held_;
  bool posture_pending_ = false;
  std::optional<int64_t> posture_request_id_;
  double posture_request_time_ = 0.0;
  std::string motion_state_ = "unknown";
  double motion_state_time_ = 0.0;
  std::optional<std::string> last_state_;
  double last_status_time_ = 0.0;

  // Gait buttons: the cycle button steps through gait_cycle_ and back to the velocity ladder (""); the hold
  // button requests gait_hold_ while held. Only in locomotion.
  std::vector<std::string> gait_cycle_;
  std::string gait_hold_;
  int gait_cycle_button_;
  int gait_hold_button_;
  size_t gait_selection_ = 0;  // 0: ladder, k: gait_cycle_[k - 1]
  std::optional<bool> gait_cycle_held_;
  bool gait_hold_armed_ = false;  // the hold button was seen released in locomotion
  std::string gait_sent_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr posture_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_state_subscription_;
  rclcpp::Publisher<ocs2_msgs::msg::WalkingVelocityCommand>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace legged_robot_teleop
