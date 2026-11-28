#include "legged_robot_teleop/base_twist_teleop.hpp"

#include <chrono>
#include <cstdio>
#include <functional>
#include <sstream>
#include <stdexcept>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace legged_robot_teleop
{
namespace
{

rclcpp::QoS bestEffort()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
}

std::string jsonString(const std::string & value)
{
  std::string out = "\"";
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out + "\"";
}

}  // namespace

BaseTwistTeleop::BaseTwistTeleop(const rclcpp::NodeOptions & options)
: Node("base_twist_teleop", options)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto joy_topic = declare_parameter<std::string>("joy_topic", "/joy", descriptor);
  const auto joy_source_topic = declare_parameter<std::string>("joy_source_topic", "", descriptor);
  const auto command_topic = declare_parameter<std::string>(
    "command_topic", "/humanoid/walking_velocity_command", descriptor);
  const auto orientation_topic = declare_parameter<std::string>(
    "orientation_topic", "/humanoid/base_orientation_command", descriptor);
  const auto posture_controller = declare_parameter<std::string>(
    "posture_controller", "", descriptor);
  posture_button_ = static_cast<int>(declare_parameter<int64_t>("button_posture", 2, descriptor));
  if (posture_button_ < 0) {
    throw std::invalid_argument("button_posture must be nonnegative");
  }

  const auto gait_topic = declare_parameter<std::string>("gait_topic", "", descriptor);
  const auto gait_cycle = declare_parameter<std::string>("gait_cycle", "", descriptor);
  gait_hold_ = declare_parameter<std::string>("gait_hold", "", descriptor);
  gait_cycle_button_ = static_cast<int>(declare_parameter<int64_t>("button_gait_cycle", 1, descriptor));
  gait_hold_button_ = static_cast<int>(declare_parameter<int64_t>("button_gait_hold", 3, descriptor));
  std::stringstream names(gait_cycle);
  for (std::string name; std::getline(names, name, ',');) {
    if (!name.empty()) {
      gait_cycle_.push_back(name);
    }
  }
  if (!gait_topic.empty()) {
    if (gait_cycle_button_ < 0 || gait_hold_button_ < 0 || gait_cycle_button_ == gait_hold_button_ ||
      gait_cycle_button_ == posture_button_ || gait_hold_button_ == posture_button_)
    {
      throw std::invalid_argument("gait buttons must be distinct, nonnegative and not the posture button");
    }
    gait_publisher_ = create_publisher<std_msgs::msg::String>(gait_topic, rclcpp::QoS(1).reliable());
  }

  if (!posture_controller.empty()) {
    std::string ns = posture_controller;
    while (!ns.empty() && ns.back() == '/') {
      ns.pop_back();
    }
    posture_client_ = create_client<std_srvs::srv::Trigger>(ns + "/toggle_stand");
    motion_state_subscription_ = create_subscription<std_msgs::msg::String>(
      ns + "/motion_state", bestEffort(),
      std::bind(&BaseTwistTeleop::receiveMotionState, this, std::placeholders::_1));
  }

  Settings s;
  const auto integer = [&](const char * name, int & value) {
      value = static_cast<int>(declare_parameter<int64_t>(name, value, descriptor));
    };
  const auto real = [&](const char * name, double & value) {
      value = declare_parameter<double>(name, value, descriptor);
    };
  integer("axis_forward", s.axis_forward);
  integer("axis_lateral", s.axis_lateral);
  integer("axis_yaw", s.axis_yaw);
  integer("axis_height", s.axis_height);
  integer("axis_enable", s.axis_enable);
  real("sign_forward", s.sign_forward);
  real("sign_lateral", s.sign_lateral);
  real("sign_yaw", s.sign_yaw);
  real("sign_height", s.sign_height);
  real("sign_enable", s.sign_enable);
  real("scale_forward", s.scale_forward);
  real("scale_lateral", s.scale_lateral);
  real("scale_yaw", s.scale_yaw);
  real("enable_threshold", s.enable_threshold);
  real("release_threshold", s.release_threshold);
  integer("button_pitch_forward", s.button_pitch_forward);
  integer("button_pitch_backward", s.button_pitch_backward);
  integer("button_center", s.button_center);
  integer("button_roll_left", s.button_roll_left);
  integer("button_roll_right", s.button_roll_right);
  real("reference_height", s.reference_height);
  real("minimum_height", s.minimum_height);
  real("maximum_height", s.maximum_height);
  real("height_rate", s.height_rate);
  real("height_axis_threshold", s.height_axis_threshold);
  real("maximum_roll", s.maximum_roll);
  real("roll_rate", s.roll_rate);
  real("maximum_pitch", s.maximum_pitch);
  real("pitch_rate", s.pitch_rate);
  real("neutral_threshold", s.neutral_threshold);
  real("stick_exponent", s.stick_exponent);
  real("publish_rate", s.publish_rate);
  control_ = std::make_unique<TeleopControl>(s);

  publisher_ = create_publisher<ocs2_msgs::msg::WalkingVelocityCommand>(command_topic, bestEffort());
  if ((s.maximum_roll > 0.0 || s.maximum_pitch > 0.0) && !orientation_topic.empty()) {
    orientation_publisher_ = create_publisher<geometry_msgs::msg::Vector3>(orientation_topic, bestEffort());
  }
  const std::string source_topic = joy_source_topic.empty() ? joy_topic : joy_source_topic;
  const auto topics = get_node_topics_interface();
  if (topics->resolve_topic_name(source_topic) != topics->resolve_topic_name(joy_topic)) {
    joy_publisher_ = create_publisher<sensor_msgs::msg::Joy>(joy_topic, bestEffort());
  }
  subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    source_topic, bestEffort(),
    std::bind(&BaseTwistTeleop::receive, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>("~/status", bestEffort());
  // Wall (steady) timer: publishing continues while simulation time is paused.
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / s.publish_rate),
    std::bind(&BaseTwistTeleop::publish, this));
  RCLCPP_INFO(
    get_logger(),
    "Output: %s. Disable other command publishers. Release the enable trigger, center sticks, "
    "then hold it to drive (DualSense default: L2). Release L2 to stop motion.",
    publisher_->get_topic_name());
}

double BaseTwistTeleop::monotonic()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void BaseTwistTeleop::receive(const sensor_msgs::msg::Joy::ConstSharedPtr & message)
{
  if (joy_publisher_) {
    joy_publisher_->publish(*message);
  }
  // float32 axes widened to double, as rclpy did.
  control_->receive(
    std::vector<double>(message->axes.begin(), message->axes.end()), message->buttons, monotonic());
  if (gait_publisher_) {
    receiveGaitButtons(*message);
  }
  if (posture_client_) {
    receivePostureButton(*message);
    if (motion_state_ != "locomotion") {
      control_->stop("robot_" + motion_state_);
    }
  }
}

void BaseTwistTeleop::receiveMotionState(const std_msgs::msg::String::ConstSharedPtr & message)
{
  motion_state_ = message->data;
  motion_state_time_ = monotonic();
  if (message->data != "locomotion") {
    control_->stop("robot_" + message->data);
    control_->resetOrientation();
  }
}

void BaseTwistTeleop::receivePostureButton(const sensor_msgs::msg::Joy & message)
{
  const std::string & state = control_->state();
  if (state == "invalid_input" || state == "invalid_time" ||
    static_cast<int>(message.buttons.size()) <= posture_button_)
  {
    posture_button_held_.reset();
    return;
  }
  const bool held = message.buttons[posture_button_] != 0;
  const bool edge = posture_button_held_.has_value() && !*posture_button_held_ && held;
  posture_button_held_ = held;
  const Settings & s = control_->settings();
  if (!edge || control_->lastTriggerPressure() > s.release_threshold) {
    return;
  }
  const auto button = [&](int i) {
      return i >= 0 && i < static_cast<int>(message.buttons.size()) && message.buttons[i] != 0;
    };
  if (button(s.button_center) ||
    (s.maximum_pitch > 0.0 && (button(s.button_pitch_forward) || button(s.button_pitch_backward))) ||
    (s.maximum_roll > 0.0 && (button(s.button_roll_left) || button(s.button_roll_right))))
  {
    return;
  }
  for (const int i : {s.axis_forward, s.axis_lateral, s.axis_yaw}) {
    if (i >= 0 && std::abs(message.axes[i]) > s.neutral_threshold) {
      return;
    }
  }
  if (s.axis_height >= 0 && std::abs(message.axes[s.axis_height]) > s.height_axis_threshold) {
    return;
  }
  if (motion_state_ != "idle" && motion_state_ != "locomotion") {
    return;
  }
  if (monotonic() - motion_state_time_ > 0.5) {
    return;
  }
  if (posture_pending_) {
    return;
  }
  if (!posture_client_->service_is_ready()) {
    RCLCPP_WARN(
      get_logger(), "Stand/sit service is unavailable; release the posture button and retry.");
    return;
  }
  control_->stop("posture_requested");
  posture_request_time_ = monotonic();
  posture_pending_ = true;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  const auto future_and_id = posture_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      posture_pending_ = false;
      posture_request_id_.reset();
      try {
        RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "%s", error.what());
      }
    });
  posture_request_id_ = future_and_id.request_id;
}

void BaseTwistTeleop::publishGait(const std::string & gait)
{
  if (gait == gait_sent_) {
    return;
  }
  std_msgs::msg::String message;
  message.data = gait;
  gait_publisher_->publish(message);
  gait_sent_ = gait;
  RCLCPP_INFO(get_logger(), "Gait: %s", gait.empty() ? "ladder (wheel_drive)" : gait.c_str());
}

void BaseTwistTeleop::receiveGaitButtons(const sensor_msgs::msg::Joy & message)
{
  const auto held = [&](int i) {
      return i < static_cast<int>(message.buttons.size()) && message.buttons[i] != 0;
    };
  const bool cycle = held(gait_cycle_button_);
  const bool hold = held(gait_hold_button_);
  // A button held at connection, or while not in locomotion, counts only after a release.
  const bool cycle_edge = gait_cycle_held_.has_value() && !*gait_cycle_held_ && cycle;
  gait_cycle_held_ = cycle;
  if (motion_state_ != "locomotion" && posture_client_) {
    gait_selection_ = 0;
    gait_hold_armed_ = false;
    publishGait("");
    return;
  }
  if (!hold) {
    gait_hold_armed_ = true;
  }
  if (cycle_edge && !gait_cycle_.empty()) {
    gait_selection_ = (gait_selection_ + 1) % (gait_cycle_.size() + 1);
  }
  const std::string selected = gait_selection_ == 0 ? std::string() : gait_cycle_[gait_selection_ - 1];
  publishGait(hold && gait_hold_armed_ && !gait_hold_.empty() ? gait_hold_ : selected);
}

void BaseTwistTeleop::stopAndPublish(const std::string & reason)
{
  control_->stop(reason);
  publish();
}

void BaseTwistTeleop::publish()
{
  const double now = monotonic();
  if (posture_pending_ && now - posture_request_time_ > 2.0) {
    if (posture_request_id_) {
      posture_client_->remove_pending_request(*posture_request_id_);
    }
    posture_pending_ = false;
    posture_request_id_.reset();
    RCLCPP_WARN(get_logger(), "Stand/sit request timed out; check robot state before retrying.");
  }
  if (posture_client_ && now - motion_state_time_ > 0.5) {
    motion_state_ = "unknown";
    control_->stop("robot_state_timeout");
  }
  const Command command = control_->command(now);
  ocs2_msgs::msg::WalkingVelocityCommand message;
  message.linear_velocity_x = command.forward;
  message.linear_velocity_y = command.lateral;
  message.angular_velocity_z = command.yaw;
  message.desired_pelvis_height = command.height;
  publisher_->publish(message);
  if (orientation_publisher_) {
    geometry_msgs::msg::Vector3 orientation;
    orientation.x = command.roll;
    orientation.y = command.pitch;
    orientation_publisher_->publish(orientation);
  }

  const std::string & state = control_->state();
  const bool changed = !last_state_ || *last_state_ != state;
  if (changed || now - last_status_time_ >= 1.0) {
    char pressure[32];
    std::snprintf(pressure, sizeof(pressure), "%.17g", control_->lastTriggerPressure());
    std_msgs::msg::String status;
    status.data = std::string("{\"phase\": ") + jsonString(toString(control_->enableState())) +
      ", \"reason\": " + jsonString(state) +
      ", \"enabled\": " + (control_->enabled() ? "true" : "false") +
      ", \"last_trigger_pressure\": " + pressure + "}";
    status_publisher_->publish(status);
    last_status_time_ = now;
  }
  if (changed) {
    std::string hint;
    if (motion_state_ == "fallen" || motion_state_ == "FAULT (damping)") {
      hint = " Robot is latched; restore a supported idle pose before resetting the controller.";
    } else if (posture_client_ && motion_state_ != "locomotion") {
      hint = " Wait for locomotion before enabling L2.";
    } else if (control_->enableState() == EnableState::WaitRelease &&
      state != "waiting_for_input")
    {
      hint = " Release L2, center sticks, release L1/D-pad, then press L2 again.";
    }
    RCLCPP_INFO(
      get_logger(), "Teleop: %s (phase=%s, L2=%.2f).%s", state.c_str(),
      toString(control_->enableState()), control_->lastTriggerPressure(), hint.c_str());
    last_state_ = state;
  }
}

}  // namespace legged_robot_teleop

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(legged_robot_teleop::BaseTwistTeleop)
