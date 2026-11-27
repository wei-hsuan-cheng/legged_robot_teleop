#include "legged_robot_teleop/control.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace legged_robot_teleop
{
namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

}  // namespace

void Settings::validate() const
{
  const std::pair<const char *, int> indices[] = {
    {"axis_forward", axis_forward}, {"axis_lateral", axis_lateral}, {"axis_yaw", axis_yaw},
    {"axis_enable", axis_enable}, {"button_height_up", button_height_up},
    {"button_height_down", button_height_down}, {"button_center", button_center},
    {"button_roll_left", button_roll_left}, {"button_roll_right", button_roll_right},
  };
  for (const auto & [name, value] : indices) {
    require(value >= -1, std::string(name) + " must be an integer >= -1");
  }
  const double reals[] = {
    sign_forward, sign_lateral, sign_yaw, sign_enable, scale_forward, scale_lateral, scale_yaw,
    enable_threshold, release_threshold, reference_height, minimum_height, maximum_height,
    height_rate, neutral_threshold, stick_exponent, publish_rate, maximum_roll, roll_rate,
  };
  for (const double value : reals) {
    require(std::isfinite(value), "Settings must be finite");
  }
  require(axis_enable >= 0, "Enable axis cannot be disabled");
  require(
    axis_enable != axis_forward && axis_enable != axis_lateral && axis_enable != axis_yaw,
    "Enable axis must be separate from motion axes");
  require(
    0 <= release_threshold && release_threshold < enable_threshold && enable_threshold <= 1,
    "Require 0 <= release_threshold < enable_threshold <= 1");
  std::set<int> buttons;
  int assigned = 0;
  for (const int button : {button_height_up, button_height_down, button_center, button_roll_left,
      button_roll_right})
  {
    if (button >= 0) {
      buttons.insert(button);
      ++assigned;
    }
  }
  require(static_cast<int>(buttons.size()) == assigned, "Button assignments must be distinct");
  for (const double sign : {sign_forward, sign_lateral, sign_yaw, sign_enable}) {
    require(sign == 1.0 || sign == -1.0, "Axis signs must be -1 or 1");
  }
  for (const double scale : {scale_forward, scale_lateral, scale_yaw}) {
    require(0.0 <= scale && scale <= 1.0, "Normalized velocity scales must be within [0, 1]");
  }
  require(
    0 < minimum_height && minimum_height <= reference_height && reference_height <= maximum_height,
    "Require 0 < minimum_height <= reference_height <= maximum_height");
  require(height_rate > 0, "Height rate must be positive");
  require(0.0 <= maximum_roll && maximum_roll <= 1.0, "Maximum roll must be within [0, 1] rad");
  require(roll_rate > 0, "Roll rate must be positive");
  require(1.0 <= stick_exponent && stick_exponent <= 3.0, "Stick exponent must be within [1, 3]");
  require(1 <= publish_rate && publish_rate <= 200, "Publish rate must be within [1, 200] Hz");
  require(
    0 <= neutral_threshold && neutral_threshold < 1, "Neutral threshold must be within [0, 1)");
}

const char * toString(EnableState state)
{
  switch (state) {
    case EnableState::WaitRelease:
      return "wait_release";
    case EnableState::Ready:
      return "ready";
    case EnableState::Enabled:
      return "enabled";
  }
  return "unknown";
}

TeleopControl::TeleopControl(const Settings & settings)
: settings_(settings), height_(settings.reference_height)
{
  settings_.validate();
}

void TeleopControl::transition(EnableState mode, const std::string & reason)
{
  enable_state_ = mode;
  state_ = reason;
  if (mode != EnableState::Enabled) {
    axes_ = {0.0, 0.0, 0.0};
    buttons_ = {false, false, false, false};
  }
}

void TeleopControl::stop(const std::string & reason)
{
  transition(EnableState::WaitRelease, reason);
}

void TeleopControl::receive(
  const std::vector<double> & axes, const std::vector<int32_t> & buttons, double now)
{
  const Settings & s = settings_;
  if (!std::isfinite(now) || (last_input_ && now < *last_input_) ||
    (last_tick_ && now < *last_tick_))
  {
    stop("invalid_time");
    last_input_.reset();
    last_tick_.reset();
    return;
  }
  const std::array<int, 3> axis_indices{s.axis_forward, s.axis_lateral, s.axis_yaw};
  // The roll buttons need to exist only when the robot takes a roll command.
  std::vector<int> button_indices{s.button_height_up, s.button_height_down, s.button_center};
  if (s.maximum_roll > 0.0) {
    button_indices.push_back(s.button_roll_left);
    button_indices.push_back(s.button_roll_right);
  }
  const auto axis_ok = [&](int i) {return i < static_cast<int>(axes.size());};
  const auto button_ok = [&](int i) {return i < static_cast<int>(buttons.size());};
  bool valid = axis_ok(s.axis_enable) &&
    std::all_of(axis_indices.begin(), axis_indices.end(), axis_ok) &&
    std::all_of(button_indices.begin(), button_indices.end(), button_ok);
  for (const double a : axes) {
    valid = valid && std::isfinite(a) && std::abs(a) <= 1.0001;
  }
  for (const int32_t b : buttons) {
    valid = valid && (b == 0 || b == 1);
  }
  if (!valid) {
    stop("invalid_input");
    last_input_.reset();
    return;
  }

  const auto pressed = [&](int index) {return index >= 0 && buttons[index] != 0;};
  // Consume presses even while driving; never replay a held Center on release.
  const bool center_pressed = pressed(s.button_center);
  const bool center_edge = center_pressed && !center_pressed_;
  center_pressed_ = center_pressed;
  std::array<double, 3> values{};
  for (std::size_t j = 0; j < 3; ++j) {
    const int i = axis_indices[j];
    values[j] = i >= 0 ? std::clamp(axes[i], -1.0, 1.0) : 0.0;
  }
  last_input_ = now;
  const double pressure = std::clamp(s.sign_enable * axes[s.axis_enable], 0.0, 1.0);
  last_trigger_pressure_ = pressure;
  // Roll buttons count only when the robot takes a roll command.
  const bool roll_enabled = s.maximum_roll > 0.0;
  const std::array<bool, 4> height_buttons{pressed(s.button_height_up), pressed(s.button_height_down),
    roll_enabled && pressed(s.button_roll_left), roll_enabled && pressed(s.button_roll_right)};
  const std::array<double, 3> scales{s.scale_forward, s.scale_lateral, s.scale_yaw};
  bool sticks_deflected = false;
  for (std::size_t j = 0; j < 3; ++j) {
    sticks_deflected = sticks_deflected || (scales[j] > 0 && std::abs(values[j]) > s.neutral_threshold);
  }
  std::string blocker;
  if (center_pressed) {
    blocker = "center_held";
  } else if (height_buttons[0] || height_buttons[1]) {
    blocker = "height_buttons_held";
  } else if (height_buttons[2] || height_buttons[3]) {
    blocker = "roll_buttons_held";
  } else if (sticks_deflected) {
    blocker = "sticks_not_centered";
  }

  if (pressure <= s.release_threshold) {
    // A released trigger qualifies READY even during Center. A later snapshot
    // may contain both the Center release and the safe L2 press.
    transition(EnableState::Ready, blocker.empty() ? "ready" : blocker);
    if (center_edge) {
      height_ = s.reference_height;
      roll_ = 0.0;
      state_ = "centered";
    }
    return;
  }

  if (enable_state_ == EnableState::WaitRelease) {
    if (state_ == "waiting_for_input") {
      state_ = "release_trigger_to_enable";
    }
    return;
  }

  if (enable_state_ == EnableState::Ready) {
    if (pressure < s.enable_threshold) {
      state_ = blocker.empty() ? "ready" : blocker;
      return;
    }
    if (!blocker.empty()) {
      stop("enable_blocked_" + blocker);
      return;
    }
    transition(EnableState::Enabled, "enabled");
    last_tick_ = now;
  }
  axes_ = values;
  buttons_ = height_buttons;
  state_ = "enabled";
}

Command TeleopControl::command(double now)
{
  const Settings & s = settings_;
  if (!std::isfinite(now) || (last_tick_ && now < *last_tick_) ||
    (last_input_ && now < *last_input_))
  {
    stop("invalid_time");
    last_input_.reset();
    last_tick_.reset();
    return {0.0, 0.0, 0.0, height_, roll_};
  }
  const double dt = last_tick_ ? std::max(0.0, now - *last_tick_) : 0.0;
  last_tick_ = now;
  if (!enabled()) {
    return {0.0, 0.0, 0.0, height_, roll_};
  }
  // A delayed callback must not cause a large jump in the height target.
  const int direction = static_cast<int>(buttons_[0]) - static_cast<int>(buttons_[1]);
  height_ += direction * s.height_rate * std::min(dt, 2.0 / s.publish_rate);
  height_ = std::clamp(height_, s.minimum_height, s.maximum_height);
  // D-pad right leans the trunk right (positive roll about the forward axis), left leans it left.
  const int roll_direction = static_cast<int>(buttons_[3]) - static_cast<int>(buttons_[2]);
  roll_ += roll_direction * s.roll_rate * std::min(dt, 2.0 / s.publish_rate);
  roll_ = std::clamp(roll_, -s.maximum_roll, s.maximum_roll);
  const auto shape = [&](double a) {return std::copysign(std::pow(std::abs(a), s.stick_exponent), a);};
  return {
    shape(axes_[0]) * s.sign_forward * s.scale_forward,
    shape(axes_[1]) * s.sign_lateral * s.scale_lateral,
    shape(axes_[2]) * s.sign_yaw * s.scale_yaw,
    height_,
    roll_,
  };
}

}  // namespace legged_robot_teleop
