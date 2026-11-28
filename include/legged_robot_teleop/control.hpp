// ROS-independent input validation, deadman control, and height integration.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace legged_robot_teleop
{

/// Immutable command mapping and limits; velocities are normalized fractions.
struct Settings
{
  int axis_forward = 1;
  int axis_lateral = 0;
  int axis_yaw = 2;
  int axis_pitch = 3;
  int axis_enable = 4;
  double sign_forward = 1.0;
  double sign_lateral = 1.0;
  double sign_yaw = 1.0;
  double sign_pitch = 1.0;
  double sign_enable = -1.0;
  double scale_forward = 0.25;
  double scale_lateral = 0.25;
  double scale_yaw = 0.25;
  double enable_threshold = 0.5;
  double release_threshold = 0.2;
  int button_height_up = 11;
  int button_height_down = 12;
  int button_center = 9;
  int button_roll_left = 13;
  int button_roll_right = 14;
  double reference_height = 0.6;
  double minimum_height = 0.2;
  double maximum_height = 1.0;
  double height_rate = 0.05;
  double maximum_roll = 0.0;  // [rad]; 0 disables the roll buttons (the robot has no commanded roll).
  double roll_rate = 0.3;  // [rad/s]
  double maximum_pitch = 0.0;  // [rad]; 0 ignores the pitch axis.
  double pitch_rate = 0.3;  // [rad/s] at full stick deflection
  double neutral_threshold = 0.1;
  double stick_exponent = 1.0;  // sign(x)|x|^e on the motion axes; 1 is linear.
  double publish_rate = 50.0;

  /// Throw std::invalid_argument on an inconsistent mapping or limit.
  void validate() const;
};

/// Normalized velocities, absolute base height in meters and trunk roll/pitch in radians.
struct Command
{
  double forward;
  double lateral;
  double yaw;
  double height;
  double roll;
  double pitch;
};

/// Single source of truth for trigger permission, separate from status reasons.
enum class EnableState { WaitRelease, Ready, Enabled };

const char * toString(EnableState state);

/// Require a neutral release/press sequence before motion. No input timeout, as
/// dric_joystick: the last sample holds until the next one.
class TeleopControl
{
public:
  explicit TeleopControl(const Settings & settings);

  const Settings & settings() const {return settings_;}
  EnableState enableState() const {return enable_state_;}
  /// Motion permission is derived from the FSM rather than a second flag.
  bool enabled() const {return enable_state_ == EnableState::Enabled;}
  const std::string & state() const {return state_;}
  double height() const {return height_;}
  double roll() const {return roll_;}
  double pitch() const {return pitch_;}

  /// Level the trunk (e.g. while the robot is not in locomotion); the height is kept.
  void resetOrientation() {roll_ = 0.0; pitch_ = 0.0;}
  double lastTriggerPressure() const {return last_trigger_pressure_;}

  /// Clear permission to move without changing the selected height.
  void stop(const std::string & reason = "disabled");

  /// Validate input and update the enable state using monotonic receipt time [s].
  void receive(const std::vector<double> & axes, const std::vector<int32_t> & buttons, double now);

  /// Return a safe command even when the ROS simulation clock is paused.
  Command command(double now);

private:
  void transition(EnableState mode, const std::string & reason);

  Settings settings_;
  double height_;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  std::string state_ = "waiting_for_input";
  EnableState enable_state_ = EnableState::WaitRelease;
  double last_trigger_pressure_ = 0.0;
  std::optional<double> last_input_;
  std::optional<double> last_tick_;
  std::array<double, 4> axes_{0.0, 0.0, 0.0, 0.0};
  std::array<bool, 4> buttons_{false, false, false, false};  // height up/down, roll left/right
  bool center_pressed_ = false;
};

}  // namespace legged_robot_teleop
