// Exercise command safety and the enable FSM without ROS, hardware, or a running simulation.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <initializer_list>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "legged_robot_teleop/control.hpp"

using legged_robot_teleop::Command;
using legged_robot_teleop::EnableState;
using legged_robot_teleop::Settings;
using legged_robot_teleop::TeleopControl;

namespace
{

struct Sample
{
  std::vector<double> axes;
  std::vector<int32_t> buttons;
};

/// Standardized SDL buttons and a ROS-sign L2 trigger axis (0 released, -1 pressed).
Sample sample(
  std::initializer_list<int> pressed = {}, std::vector<double> axes = std::vector<double>(6, 0.0),
  double pressure = 0.0)
{
  axes[4] = -pressure;
  std::vector<int32_t> buttons(15, 0);
  for (const int index : pressed) {
    buttons[index] = 1;
  }
  return {axes, buttons};
}

std::vector<double> stick(double forward, double lateral = 0.0, double yaw = 0.0)
{
  return {lateral, forward, yaw, 0.0, 0.0, 0.0};
}

void receive(TeleopControl & control, const Sample & s, double now)
{
  control.receive(s.axes, s.buttons, now);
}

/// Release the deadman and then press it with centered controls.
void arm(TeleopControl & control, double now = 0.0)
{
  receive(control, sample(), now);
  receive(control, sample({}, stick(0), 1.0), now + 0.01);
  ASSERT_TRUE(control.enabled());
}

Settings with(const std::function<void(Settings &)> & change)
{
  Settings s;
  change(s);
  return s;
}

/// A complete game-controller snapshot rather than separate events.
void send(
  TeleopControl & control, double now, double pressure = 0.0, double forward = 0.0,
  std::initializer_list<int> buttons = {})
{
  receive(control, sample(buttons, stick(forward), pressure), now);
}

bool zeroVelocity(const Command & c) {return c.forward == 0 && c.lateral == 0 && c.yaw == 0;}

}  // namespace

// --- Mapping, scaling and height ---------------------------------------------------------------

TEST(Control, StartupRequiresReleaseAndNeutralPress)
{
  TeleopControl control{Settings{}};
  EXPECT_EQ(control.command(0).forward, 0);
  receive(control, sample({}, stick(0), 1.0), 0.0);
  EXPECT_FALSE(control.enabled());
  receive(control, sample(), 0.01);
  receive(control, sample({}, stick(1), 1.0), 0.02);
  EXPECT_FALSE(control.enabled());
  receive(control, sample({}, stick(0), 1.0), 0.03);
  EXPECT_FALSE(control.enabled());
  arm(control, 0.04);
}

TEST(Control, AxesAreNormalizedWithRosSigns)
{
  TeleopControl control{with([](Settings & s) {
      s.scale_forward = 0.4; s.scale_lateral = 0.2; s.scale_yaw = 0.3;
    })};
  arm(control);
  receive(control, sample({}, stick(1.0, -0.5, -1.0), 1.0), 0.02);
  const Command command = control.command(0.03);
  EXPECT_DOUBLE_EQ(command.forward, 0.4);
  EXPECT_DOUBLE_EQ(command.lateral, -0.1);
  EXPECT_DOUBLE_EQ(command.yaw, -0.3);
}

TEST(Control, ReleaseZeroesVelocitiesPreservingHeight)
{
  TeleopControl control{Settings{}};
  arm(control);
  receive(control, sample({11}, stick(0), 1.0), 0.02);
  const double height = control.command(0.04).height;
  receive(control, sample(), 0.05);
  const Command command = control.command(0.06);
  EXPECT_TRUE(zeroVelocity(command));
  EXPECT_EQ(command.height, height);
  receive(control, sample({}, stick(0), 1.0), 0.07);
  EXPECT_TRUE(control.enabled());
}

TEST(Control, CircleIsUnboundDuringArmingAndDriving)
{
  TeleopControl control{Settings{}};
  receive(control, sample({1}, stick(0), 1.0), 0.0);
  EXPECT_FALSE(control.enabled()) << "Circle must not bypass startup release";
  receive(control, sample({1}), 0.01);
  receive(control, sample({1}, stick(0), 1.0), 0.02);
  EXPECT_TRUE(control.enabled()) << "Holding Circle must not block a safe L2 press";
  receive(control, sample({1}, stick(1), 1.0), 0.03);
  EXPECT_EQ(control.command(0.04).forward, control.settings().scale_forward);
  receive(control, sample({}, stick(0), 1.0), 0.05);
  EXPECT_TRUE(control.enabled()) << "Releasing Circle must not disarm driving";
  receive(control, sample({1}), 0.06);
  EXPECT_FALSE(control.enabled()) << "L2 release must still stop motion";
}

TEST(Control, SilentInputHoldsLastCommandUntilReleased)
{
  TeleopControl control{Settings{}};
  arm(control);
  receive(control, sample({}, stick(1), 1.0), 0.02);
  EXPECT_EQ(control.command(10.0).forward, control.settings().scale_forward);
  EXPECT_TRUE(control.enabled());
  receive(control, sample({}, stick(1)), 10.01);
  EXPECT_EQ(control.command(10.02).forward, 0);
}

TEST(Control, StickExponentShapesMotionAxesOnly)
{
  TeleopControl control{with([](Settings & s) {
      s.stick_exponent = 2.0; s.scale_forward = 1.0; s.scale_yaw = 1.0;
    })};
  arm(control);
  receive(control, sample({}, stick(0.5, 0.0, -0.5), 1.0), 0.02);
  const Command command = control.command(0.03);
  EXPECT_NEAR(command.forward, 0.25, 1e-12);
  EXPECT_NEAR(command.yaw, -0.25, 1e-12);
  receive(control, sample({}, stick(1.0), 1.0), 0.04);
  EXPECT_NEAR(control.command(0.05).forward, 1.0, 1e-12);
}

TEST(Control, InvalidInputDisarms)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<Sample> cases{
    {{}, std::vector<int32_t>(15, 0)},
    {std::vector<double>(6, 0.0), {}},
    {std::vector<double>(6, nan), std::vector<int32_t>(15, 0)},
    {std::vector<double>(6, inf), std::vector<int32_t>(15, 0)},
    {std::vector<double>(6, 1.1), std::vector<int32_t>(15, 0)},
    {std::vector<double>(6, 0.0), std::vector<int32_t>(15, 2)},
    {std::vector<double>(4, 0.0), std::vector<int32_t>(15, 0)},  // missing enable axis
  };
  for (const Sample & bad : cases) {
    TeleopControl control{Settings{}};
    arm(control);
    receive(control, bad, 0.02);
    EXPECT_EQ(control.state(), "invalid_input");
    EXPECT_EQ(control.command(0.03).forward, 0);
    receive(control, sample({}, stick(0), 1.0), 0.04);
    EXPECT_FALSE(control.enabled());
  }
}

TEST(Control, HeightLimitsAndCenter)
{
  TeleopControl control{with([](Settings & s) {
      s.reference_height = 0.22; s.minimum_height = 0.2; s.maximum_height = 0.33;
      s.height_rate = 0.5;
    })};
  arm(control);
  for (int i = 1; i < 50; ++i) {
    receive(control, sample({11}, stick(0), 1.0), 0.02 * i);
    control.command(0.02 * i);
  }
  EXPECT_EQ(control.height(), 0.33);
  for (int i = 50; i < 100; ++i) {
    receive(control, sample({12}, stick(0), 1.0), 0.02 * i);
    control.command(0.02 * i);
  }
  EXPECT_EQ(control.height(), 0.2);
  receive(control, sample({9}), 2.0);
  const Command command = control.command(2.0);
  EXPECT_EQ(command.height, 0.22);
  EXPECT_TRUE(zeroVelocity(command));
  EXPECT_FALSE(control.enabled());
}

TEST(Control, RollLimitsCenterAndReset)
{
  TeleopControl control{with([](Settings & s) {s.maximum_roll = 0.25; s.roll_rate = 0.5;})};
  arm(control);
  // D-pad right (14) leans right, up to +maximum_roll.
  for (int i = 1; i < 50; ++i) {
    receive(control, sample({14}, stick(0), 1.0), 0.02 * i);
    control.command(0.02 * i);
  }
  EXPECT_DOUBLE_EQ(control.roll(), 0.25);
  // D-pad left (13) leans left, down to -maximum_roll; the height does not move.
  for (int i = 50; i < 150; ++i) {
    receive(control, sample({13}, stick(0), 1.0), 0.02 * i);
    control.command(0.02 * i);
  }
  EXPECT_DOUBLE_EQ(control.roll(), -0.25);
  EXPECT_EQ(control.height(), 0.6);
  // Released L2 holds the roll; L1 (Center) levels it.
  receive(control, sample({}, stick(0), 0.0), 3.1);
  EXPECT_DOUBLE_EQ(control.command(3.12).roll, -0.25);
  receive(control, sample({9}), 3.2);
  EXPECT_EQ(control.command(3.22).roll, 0.0);
  arm(control, 3.3);
  receive(control, sample({14}, stick(0), 1.0), 3.4);
  control.command(3.42);
  EXPECT_GT(control.roll(), 0.0);
  control.resetRoll();
  EXPECT_EQ(control.roll(), 0.0);
}

TEST(Control, RollButtonsAreIgnoredWithoutARollLimit)
{
  TeleopControl control{Settings{}};
  arm(control);
  for (int i = 1; i < 20; ++i) {
    receive(control, sample({14}, stick(0), 1.0), 0.02 * i);
    EXPECT_TRUE(control.enabled());
    EXPECT_EQ(control.command(0.02 * i).roll, 0.0);
  }
}

TEST(Control, BothHeightButtonsCancelAndDisabledHeightIsIgnored)
{
  TeleopControl control{Settings{}};
  receive(control, sample({11}), 0.0);
  EXPECT_EQ(control.command(0.02).height, 0.6);
  arm(control, 0.03);
  receive(control, sample({11, 12}, stick(0), 1.0), 0.05);
  EXPECT_EQ(control.command(0.06).height, 0.6);
}

TEST(Control, HeightDoesNotJumpAfterADelayedCallback)
{
  TeleopControl control{Settings{}};
  arm(control);
  receive(control, sample({11}, stick(0), 1.0), 0.2);
  EXPECT_LE(control.command(0.21).height - 0.6, 0.05 * 0.04 + 1e-12);
}

TEST(Control, UpkieIgnoresLateralAxisAndOptionalButtonsCanBeDisabled)
{
  TeleopControl control{with([](Settings & s) {
      s.scale_lateral = 0.0; s.axis_lateral = -1; s.button_center = -1;
    })};
  arm(control);
  receive(control, sample({}, stick(0.5, 1.0), 1.0), 0.02);
  const Command command = control.command(0.03);
  EXPECT_EQ(command.forward, 0.125);
  EXPECT_EQ(command.lateral, 0);
}

TEST(Control, InvalidSettingsFailFast)
{
  const std::vector<std::function<void(Settings &)>> changes{
    [](Settings & s) {s.minimum_height = 0.0;},
    [](Settings & s) {s.reference_height = 2.0;},
    [](Settings & s) {s.height_rate = -0.1;},
    [](Settings & s) {s.stick_exponent = 0.5;},
    [](Settings & s) {s.stick_exponent = std::numeric_limits<double>::infinity();},
    [](Settings & s) {s.scale_forward = 1.1;},
    [](Settings & s) {s.scale_yaw = std::numeric_limits<double>::quiet_NaN();},
    [](Settings & s) {s.axis_enable = -1;},
    [](Settings & s) {s.button_center = 11;},
    [](Settings & s) {s.axis_forward = -2;},
    [](Settings & s) {s.sign_forward = 0.0;},
    [](Settings & s) {s.publish_rate = 0.0;},
    [](Settings & s) {s.neutral_threshold = 1.0;},
    [](Settings & s) {s.enable_threshold = 0.2;},
    [](Settings & s) {s.enable_threshold = 1.1;},
    [](Settings & s) {s.release_threshold = -0.1;},
    [](Settings & s) {s.axis_enable = 1;},
    [](Settings & s) {s.sign_enable = 0.0;},
  };
  for (std::size_t i = 0; i < changes.size(); ++i) {
    EXPECT_THROW(TeleopControl{with(changes[i])}, std::invalid_argument) << "case " << i;
  }
}

TEST(Control, L1DoesNotEnableMotion)
{
  TeleopControl control{Settings{}};
  receive(control, sample(), 0.0);
  receive(control, sample({9}), 0.01);
  EXPECT_FALSE(control.enabled());
  EXPECT_EQ(control.command(0.02).forward, 0);
}

TEST(Control, TriggerThresholdHysteresisAndGradualPress)
{
  TeleopControl control{Settings{}};
  receive(control, sample(), 0.0);
  receive(control, sample({}, stick(0), 0.3), 0.01);
  EXPECT_FALSE(control.enabled());
  receive(control, sample({}, stick(0), 0.5), 0.02);
  EXPECT_TRUE(control.enabled());
  receive(control, sample({}, stick(0), 0.3), 0.03);
  EXPECT_TRUE(control.enabled());
  receive(control, sample({}, stick(0), 0.2), 0.04);
  EXPECT_FALSE(control.enabled());
  EXPECT_EQ(control.command(0.05).forward, 0);
}

TEST(Control, PartlyHeldTriggerCannotRearmAfterStop)
{
  TeleopControl control{Settings{}};
  arm(control, 1.03);
  control.stop("robot_inactive");
  receive(control, sample({}, stick(0), 0.3), 1.06);
  receive(control, sample({}, stick(0), 1.0), 1.07);
  EXPECT_FALSE(control.enabled());
}

TEST(Control, CenterIsIgnoredWhileEnabledAndNotQueuedUntilRelease)
{
  TeleopControl control{Settings{}};
  arm(control);
  receive(control, sample({11}, stick(0), 1.0), 0.02);
  const double height = control.command(0.04).height;
  EXPECT_GT(height, control.settings().reference_height);
  receive(control, sample({9}, stick(1), 1.0), 0.05);
  Command command = control.command(0.06);
  EXPECT_EQ(command.forward, 0.25);
  EXPECT_EQ(command.height, height);
  receive(control, sample({9}), 0.07);
  command = control.command(0.08);
  EXPECT_EQ(command.forward, 0);
  EXPECT_EQ(command.height, height);
  receive(control, sample(), 0.09);
  receive(control, sample({9}), 0.1);
  EXPECT_EQ(control.command(0.11).height, control.settings().reference_height);
}

TEST(Control, CenterHeldCannotEnableMotion)
{
  TeleopControl control{Settings{}};
  receive(control, sample({9}), 0.0);
  receive(control, sample({9}, stick(0), 1.0), 0.01);
  EXPECT_FALSE(control.enabled());
  receive(control, sample({}, stick(0), 1.0), 0.02);
  EXPECT_FALSE(control.enabled());
  arm(control, 0.03);
}

TEST(Control, CenterIsIgnoredWithAPartiallyPressedTrigger)
{
  TeleopControl control{Settings{}};
  arm(control);
  receive(control, sample({11}, stick(0), 1.0), 0.02);
  const double height = control.command(0.04).height;
  receive(control, sample(), 0.05);
  receive(control, sample({9}, stick(0), 0.3), 0.06);
  EXPECT_EQ(control.command(0.07).height, height);
  receive(control, sample({9}), 0.08);
  EXPECT_EQ(control.command(0.09).height, height);
}

TEST(Control, CircleDoesNotBlockCenter)
{
  TeleopControl control{Settings{}};
  arm(control);
  receive(control, sample({11}, stick(0), 1.0), 0.02);
  EXPECT_GT(control.command(0.04).height, control.settings().reference_height);
  receive(control, sample({1, 9}), 0.05);
  EXPECT_EQ(control.state(), "centered");
  receive(control, sample({9}), 0.06);
  const Command command = control.command(0.07);
  EXPECT_EQ(command.height, control.settings().reference_height);
  EXPECT_EQ(command.forward, 0);
}

TEST(Control, CenterReleaseAndL2PressCanArriveInTheSameFrame)
{
  TeleopControl control{Settings{}};
  receive(control, sample({9}), 0.0);
  EXPECT_FALSE(control.enabled());
  receive(control, sample({}, stick(0), 1.0), 0.01);
  EXPECT_TRUE(control.enabled());
}

TEST(Control, CenterReleaseDuringGradualL2PressPreservesReadyState)
{
  TeleopControl control{Settings{}};
  receive(control, sample({9}), 0.0);
  receive(control, sample({}, stick(0), 0.3), 0.01);
  EXPECT_FALSE(control.enabled());
  receive(control, sample({}, stick(0), 0.6), 0.02);
  EXPECT_TRUE(control.enabled());
}

// --- Enable FSM -------------------------------------------------------------------------------

TEST(EnableFsm, ReadySurvivesSlowTriggerSqueezeAfterCenter)
{
  TeleopControl control{Settings{}};
  send(control, 0.0, 0.0, 0.0, {9});
  EXPECT_EQ(control.enableState(), EnableState::Ready);
  int i = 1;
  for (const double pressure : {0.21, 0.3, 0.45, 0.49}) {
    send(control, 0.01 * i, pressure);
    EXPECT_EQ(control.enableState(), EnableState::Ready);
    EXPECT_EQ(control.command(0.01 * i).forward, 0);
    ++i;
  }
  send(control, 0.05, 0.5);
  EXPECT_EQ(control.enableState(), EnableState::Enabled);
}

TEST(EnableFsm, BlockedPressExplainsReasonAndRequiresRelease)
{
  struct Case
  {
    std::initializer_list<int> buttons;
    double forward;
    const char * reason;
  };
  for (const Case & c : {Case{{9}, 0.0, "center_held"}, Case{{11}, 0.0, "height_buttons_held"},
      Case{{}, 0.2, "sticks_not_centered"}})
  {
    TeleopControl control{Settings{}};
    send(control, 0.0);
    send(control, 0.01, 0.6, c.forward, c.buttons);
    EXPECT_EQ(control.enableState(), EnableState::WaitRelease);
    EXPECT_EQ(control.state(), std::string("enable_blocked_") + c.reason);
    send(control, 0.02, 0.9);
    EXPECT_EQ(control.state(), std::string("enable_blocked_") + c.reason);
    EXPECT_EQ(control.command(0.03).forward, 0);
    send(control, 0.04);
    EXPECT_EQ(control.enableState(), EnableState::Ready);
    send(control, 0.05, 0.6);
    EXPECT_TRUE(control.enabled());
  }
}

TEST(EnableFsm, ReadyCanWaitForNeutralBeforeTriggerCrossesThreshold)
{
  TeleopControl control{Settings{}};
  send(control, 0.0, 0.0, 0.5);
  EXPECT_EQ(control.enableState(), EnableState::Ready);
  send(control, 0.01, 0.4, 0.5);
  EXPECT_EQ(control.state(), "sticks_not_centered");
  send(control, 0.02, 0.45);
  EXPECT_EQ(control.state(), "ready");
  send(control, 0.03, 0.6);
  EXPECT_TRUE(control.enabled());
}

TEST(EnableFsm, EnabledHysteresisDoesNotRequireNeutralSticks)
{
  TeleopControl control{Settings{}};
  send(control, 0.0);
  send(control, 0.01, 0.6);
  int i = 2;
  for (const double pressure : {0.49, 0.3, 0.201, 0.6}) {
    send(control, 0.01 * i, pressure, 0.8, {9});
    EXPECT_TRUE(control.enabled());
    EXPECT_NEAR(control.command(0.01 * i).forward, 0.2, 1e-7);
    ++i;
  }
  send(control, 0.06, 0.2, 0.8);
  EXPECT_EQ(control.enableState(), EnableState::Ready);
  EXPECT_EQ(control.command(0.07).forward, 0);
}

TEST(EnableFsm, FaultAndCenterReleaseRecovery)
{
  for (const bool malformed : {false, true}) {
    TeleopControl control{Settings{}};
    send(control, 0.0);
    send(control, 0.01, 0.6);
    if (malformed) {
      control.receive({}, {}, 0.02);
    } else {
      control.stop("robot_inactive");
    }
    EXPECT_EQ(control.enableState(), EnableState::WaitRelease);
    send(control, 0.4, 0.6);
    EXPECT_FALSE(control.enabled());
    send(control, 0.41, 0.0, 0.0, {9});
    EXPECT_EQ(control.enableState(), EnableState::Ready);
    send(control, 0.42, 0.6);
    EXPECT_TRUE(control.enabled());
  }
}

TEST(EnableFsm, InvalidTimeDisarmsAndCanRecover)
{
  for (const double now : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -0.1})
  {
    for (const bool via_receive : {true, false}) {
      TeleopControl control{Settings{}};
      send(control, 0.0);
      send(control, 0.01, 0.6);
      if (via_receive) {
        send(control, now, 0.6);
      } else {
        EXPECT_TRUE(zeroVelocity(control.command(now)));
      }
      EXPECT_EQ(control.state(), "invalid_time");
      EXPECT_EQ(control.enableState(), EnableState::WaitRelease);
      send(control, 0.1, 0.6);
      EXPECT_FALSE(control.enabled());
      send(control, 0.11);
      send(control, 0.12, 0.6);
      EXPECT_TRUE(control.enabled());
    }
  }
}

TEST(EnableFsm, RandomizedInputSequencesNeverBypassEnableGuards)
{
  const double steps[] = {0.001, 0.01, 0.02, 0.3};
  const double pressures[] = {0.0, 0.19, 0.2, 0.21, 0.49, 0.5, 0.8, 1.0};
  const double forwards[] = {0.0, 0.05, 0.3, 1.0, -1.0};
  const std::vector<std::vector<int>> button_sets{{}, {9}, {1}, {11}, {12}, {1, 9}, {11, 12}};
  for (unsigned seed = 0; seed < 6; ++seed) {
    std::mt19937 rng(seed);
    const auto pick = [&](std::size_t n) {return std::uniform_int_distribution<std::size_t>(0, n - 1)(rng);};
    TeleopControl control{Settings{}};
    const Settings & s = control.settings();
    double now = 0.0;
    for (int k = 0; k < 1000; ++k) {
      now += steps[pick(4)];
      const EnableState previous = control.enableState();
      const double pressure = pressures[pick(8)];
      const double forward = forwards[pick(5)];
      const std::vector<int> & buttons = button_sets[pick(button_sets.size())];
      Sample snapshot = sample({}, stick(forward), pressure);
      for (const int b : buttons) {
        snapshot.buttons[b] = 1;
      }
      receive(control, snapshot, now);
      const Command command = control.command(now);
      const auto has = [&](int b) {
          for (const int x : buttons) {
            if (x == b) {return true;}
          }
          return false;
        };
      ASSERT_EQ(control.enabled(), control.enableState() == EnableState::Enabled);
      ASSERT_GE(command.height, s.minimum_height);
      ASSERT_LE(command.height, s.maximum_height);
      ASSERT_TRUE(std::isfinite(command.forward) && std::isfinite(command.lateral) &&
        std::isfinite(command.yaw) && std::isfinite(command.height));
      ASSERT_LE(std::abs(command.forward), s.scale_forward);
      if (!control.enabled()) {
        ASSERT_TRUE(zeroVelocity(command));
      }
      if (pressure <= s.release_threshold) {
        ASSERT_FALSE(control.enabled());
      }
      if (control.enabled() && previous != EnableState::Enabled) {
        ASSERT_EQ(previous, EnableState::Ready);
        ASSERT_GE(pressure, s.enable_threshold);
        ASSERT_LE(std::abs(forward), s.neutral_threshold);
        ASSERT_FALSE(has(9) || has(11) || has(12));
      }
    }
  }
}
