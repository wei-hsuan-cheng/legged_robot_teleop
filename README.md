# Legged Robot Teleop

DualSense gamepad commands for legged robot MPC, using ROS 2 Jazzy's standard `joy` driver.
The adapter is a C++ node (`base_twist`, also the `legged_robot_teleop::BaseTwistTeleop` component); a small Python module, `legged_robot_teleop.profile`, reads robot height bounds for the launch file.
Publishes the same `ocs2_msgs/msg/WalkingVelocityCommand` as the base-command GUI.
No MPC controller, estimator, robot model, or low-level control is implemented here.


## Build

Use a workspace with `ocs2_msgs` available; the robot examples also need `legged_robot_mpc_controller` installed so launch can read its robot reference settings.
The adapter does not depend on the MPC package at build time, avoiding a dependency cycle.
The MPC dependency manifest imports this repository automatically.
For a standalone checkout into an existing robot workspace:

```bash
cd <workspace>/src
git clone https://github.com/wei-hsuan-cheng/legged_robot_teleop.git
cd ..
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rosdep install --from-paths src/legged_robot_teleop --ignore-src -r -y
colcon build --symlink-install --packages-select legged_robot_teleop
source install/setup.bash
```

Do not clone a second copy if the VCS manifest already imported it.
For container builds, connect the controller to the Ubuntu VM and make its input event devices available inside the container first.
See [connection and testing](docs/testing.md) for USB, Docker, isolated topic checks, and robot tests.


## Controls

| Control | Action |
| --- | --- |
| L2 held | Enable motion after releasing L2 and centering both sticks |
| Left stick | Forward/backward and left/right |
| Right stick horizontal | Yaw rate |
| Right stick forward/back, with L2 | Increase/decrease base height; ignores the first 30% of stick travel |
| D-pad up/down, with L2 | Upkie, Pineapple: lean forward/back (pitch), up to `maxBasePitch`; holds when released |
| D-pad left/right, with L2 | Upkie, Pineapple: lean the trunk left/right (roll), up to the robot's `maxBaseRoll`; held when L2 is released |
| L1, with L2 released | Center: zero velocity/yaw, restore nominal base height and level the trunk |
| Square, with L2 released | Upkie, Pineapple: toggle Stand Up / Sit Down, with centered sticks |
| Circle, in locomotion | Upkie, Pineapple: cycle the gait walk -> trot -> wheel_drive (the velocity ladder) |
| Triangle held, in locomotion | Upkie, Pineapple: jump while held, then back to the Circle selection |
| Release L2 | Zero velocity/yaw, keep the selected height and tilt |

The launch YAML currently uses 100% of the MPC's configured maximum speeds; start with small stick deflections.
Lower the fractions in `config/dualsense.yaml` for slower joystick control without changing MPC tuning.
Upkie's robot profile disables independent sideways commands (its `maxDisplacementVelocityY` is 0); Pineapple's
enables them: a sideways command makes its MPC step (trot) with the hip ab/ad, since the tires cannot roll sideways.
The gait buttons publish the controller's `target.gaitTopic` (`/humanoid/gait_command`), only for robots with a planned
stand-up and only the templates the controller defines; leaving locomotion (sit-down, fall) returns to the ladder and
a button held at that moment counts only after a release.
For Upkie, Square requests stand-up from idle or sit-down from locomotion; intermediate and fallen/fault states refuse it.
Square also requires L1 and D-pad released.
Held Square never repeats or queues a blocked request; release and press again.
Square held at connection is ignored until released and pressed again.
The default is `button_posture: 2` (SDL X / DualSense Square); R1 no longer requests stand-up or sit-down.
The controller owns the posture transition; teleop only calls its `~/toggle_stand` service.
While the robot is not in locomotion, or its 10 Hz state heartbeat is lost for 0.5 s, the adapter disarms driving.
After stand-up, release and press L2 again to enable motion.
Posture control is enabled automatically only when the selected controller YAML defines a planned stand-up; G1/H2 have no new posture-button behavior.
There is no input timeout, as in `dric_joystick`: the last `/joy` sample holds until the next one, and releasing L2 stops motion. If the gamepad link drops while L2 is held, the last command keeps being published and the MPC keeps following it; stop the robot by other means (e.g. reconnect and release L2, or the controller/E-stop).

Velocity-stick deflection is shaped as sign(x)|x|^`stick_exponent` (2 in `dualsense.yaml`) for fine low-speed control; full deflection still commands the full scaled speed.
Height uses its own `height_axis_threshold` (0.3): within the deadband it holds; above it the rate grows linearly from zero to `height_rate` (0.2 m/s in `dualsense.yaml`).
L2 is axis 4, reported by `game_controller_node` as 0 released to -1 fully pressed.
The deadman engages at 50% of that ROS range and releases at 20%, avoiding threshold chatter.
Centered velocity sticks, the height stick inside its deadband, and released Center/active tilt buttons are required at the instant of enabling.
Center requires L2 at or below its release threshold and a fresh L1 press.
L1 presses during driving or partial trigger presses are ignored, not queued for later.
Release L1 before re-enabling L2.
L1 release and a neutral L2 press may arrive in the same input message; no extra both-released frame is required after Center.
An enable attempt with deflected sticks or held L1/D-pad is rejected until L2 is released and pressed again.
See [enable states and diagnostics](docs/enable_fsm.md) for the complete transition table and recovery instructions.
This is not a hardware emergency stop and does not deactivate the MPC.


## Run

First complete the isolated topic checks in [testing](docs/testing.md).
With the integrated MPC launch, select `base_twist` mode and run:

```bash
ros2 launch legged_robot_mpc_controller robot.launch.py \
  robot_name:=h2 mpc_freq:=80 base_command_device:=joystick
```

The `base_command_device` selector accepts `gui`, `joystick`, or `none`, so the integrated launch cannot start both command sources.
The robot's resolved controller YAML and effective nominal height are passed through automatically.
Do not also start a standalone driver or teleop node.
Alternatively, start the robot with `base_command_device:=none`, then run teleop separately:

```bash
ros2 launch legged_robot_teleop teleop.launch.py robot_name:=h2
```

Use `robot_name:=g1`, `robot_name:=upkie` or `robot_name:=pineapple` for the other profiles.
The launch file reads `defaultBaseHeight`, `minBaseHeight`, and `maxBaseHeight` directly from the selected robot's controller YAML.
If robot launch uses `controllers_file` or `ref_base_height`, pass the same values to teleop.
No robot height settings are duplicated in this package.


## Configuration

Edit [config/dualsense.yaml](config/dualsense.yaml) or provide a copy with `params_file:=/absolute/path.yaml`.
The driver is `game_controller_node`, not `joy_node`; it provides standardized SDL button/axis ordering and already uses positive forward/left ROS signs.
See the [upstream driver documentation](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md).

| Argument | Default / Purpose |
| --- | --- |
| `robot_name` | `g1`; also `h2`, `upkie`, `pineapple` (no arm), `pineapple_manipulator` (arm stowed) |
| `controllers_file` | Empty: use installed MPC `config/<robot>/ros2_controllers.yaml` |
| `mpc_controller_name` | `humanoid_whole_body_inverse_dynamics_mpc_controller`; YAML key |
| `ref_base_height` | Empty: read nominal height from controller YAML |
| `maximum_roll`, `maximum_pitch`, `orientation_topic` | From the controller YAML (`ocs2.reference.maxBaseRoll`, `maxBasePitch`, `target.baseOrientationTopic`); 0 disables that tilt axis |
| `button_roll_left`, `button_roll_right`, `roll_rate` | D-pad left 13, right 14 (SDL), 0.3 rad/s |
| `button_pitch_forward`, `button_pitch_backward`, `pitch_rate` | D-pad up 11, down 12 (SDL), 0.3 rad/s |
| `axis_height`, `sign_height`, `height_axis_threshold` | Right stick vertical 3 (SDL), +1, deadband 0.3 |
| `height_rate` | Maximum height change rate, 0.2 m/s at full stick in `dualsense.yaml` |
| `params_file` | Installed `config/dualsense.yaml` |
| `device_name` | `DualSense Wireless Controller`; takes precedence over `device_id` |
| `device_id` | `0`; clear `device_name` to select by ID |
| `start_joy` | `true`; set `false` if a driver is already publishing `joy_topic` |
| `joy_topic` | `/joy` |
| `command_topic` | `/humanoid/walking_velocity_command` |
| `use_sim_time` | `false`; watchdog and publishing use wall/steady time either way |

Output velocities are normalized fractions within `[-1, 1]`, not physical m/s or rad/s.
`desired_pelvis_height` is an absolute height in meters, matching the GUI contract.
For robots with a roll or pitch limit the node also publishes `geometry_msgs/Vector3` on `orientation_topic` (x roll [rad], positive leans right; y pitch [rad], positive leans forward in the command frame) at the same rate.
The D-pad controls tilt rates, not absolute angles: release the buttons to hold the angle, release L2 and press L1 to level both axes.
Roll and pitch also level when the robot leaves locomotion.
The height stick must be inside its deadband and active tilt buttons released before enabling L2 or toggling stand/sit; robots with `maxBasePitch: 0` ignore the pitch buttons.
How the controller turns it into leg lengths, with results and sources: legged_robot_mpc_controller `docs/base_orientation_control.md`.
The bundled driver uses `<joy_topic>/raw`; the adapter subscribes with best effort and exposes the unchanged joystick data on public `joy_topic` with best effort, volatile, depth 1.
Commands and `~/status` diagnostics also use best effort, volatile, depth 1.
With `start_joy:=false`, the adapter directly consumes the external topic using best effort; that external publisher's QoS is not changed.
Only one base-command source may publish at once; disable the GUI and scripted targets before using teleop.
Axis/button mapping and safety parameters are read-only after startup; restart to apply a new mapping.
Optional velocity/height-axis and tilt-button indices can be `-1` to disable them; the enable axis is always required.


## Tests

```bash
colcon test --packages-select legged_robot_teleop --event-handlers console_direct+
colcon test-result --verbose
```

C++ unit tests (gtest, `test/test_control.cpp`) cover signs/scaling, height bounds, release/stop, malformed input, held commands without new input, the stick response curve, the D-pad roll (limits, hold on release, Center, ignored without a roll limit), and the enable FSM: post-Center re-enabling, trigger hysteresis, blocked-input recovery, clock faults, and 6,000 deterministic randomized input steps.
Pitch tests cover both limits, opposing buttons, hold on release, L1/reset and optional button validation; height tests cover the deadband boundary, proportional rates, limits, direction, disabled/missing axes and arming guards.
A DDS test checks pitch-only output, best-effort QoS and leveling outside locomotion.
The ROS-level pytest tests start the built `base_twist` executable and observe it only through topics, `~/status` and the ROS graph.
Isolated service tests cover Square edges, configurable mapping, blocked presses, transition states, stale robot status and L2 re-arming at handover.
The ROS transport test uses its own ROS domain and unique test topics, including a paused simulation clock with no `/clock` publisher.
It never sends robot commands or starts a simulation.
