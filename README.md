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
| D-pad up/down, with L2 | Increase/decrease base height |
| L1, with L2 released | Center: zero velocity/yaw and restore nominal base height |
| Square, with L2 released | Upkie: toggle Stand Up / Sit Down, with centered sticks |
| Circle | Unassigned; reserved for future use |
| Release L2 or lose input | Zero velocity/yaw, keep the selected height |

The launch YAML currently uses 100% of the MPC's configured maximum speeds; start with small stick deflections.
Lower the fractions in `config/dualsense.yaml` for slower joystick control without changing MPC tuning.
Upkie's robot profile disables independent sideways commands.
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

Stick deflection is shaped as sign(x)|x|^`stick_exponent` (2 in `dualsense.yaml`) for fine low-speed control; full deflection still commands the full scaled speed.
L2 is axis 4, reported by `game_controller_node` as 0 released to -1 fully pressed.
The deadman engages at 50% of that ROS range and releases at 20%, avoiding threshold chatter.
Centered sticks and released Center/height buttons are required at the instant of enabling.
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
  robotName:=h2 mpcFreq:=80 baseCommandDevice:=joystick
```

The `baseCommandDevice` selector accepts `gui`, `joystick`, or `none`, so the integrated launch cannot start both command sources.
The robot's resolved controller YAML and effective nominal height are passed through automatically.
Do not also start a standalone driver or teleop node.
Alternatively, start the robot with `baseCommandDevice:=none`, then run teleop separately:

```bash
ros2 launch legged_robot_teleop teleop.launch.py robotName:=h2
```

Use `robotName:=g1`, `robotName:=upkie` or `robotName:=pineapple` for the other profiles.
The launch file reads `defaultBaseHeight`, `minBaseHeight`, and `maxBaseHeight` directly from the selected robot's controller YAML.
If robot launch uses `controllersFile` or `refBaseHeight`, pass the same values to teleop.
No robot height settings are duplicated in this package.


## Configuration

Edit [config/dualsense.yaml](config/dualsense.yaml) or provide a copy with `paramsFile:=/absolute/path.yaml`.
The driver is `game_controller_node`, not `joy_node`; it provides standardized SDL button/axis ordering and already uses positive forward/left ROS signs.
See the [upstream driver documentation](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md).

| Argument | Default / Purpose |
| --- | --- |
| `robotName` | `g1`; also `h2`, `upkie`, `pineapple` |
| `controllersFile` | Empty: use installed MPC `config/<robot>/ros2_controllers.yaml` |
| `mpcControllerName` | `humanoid_whole_body_inverse_dynamics_mpc_controller`; YAML key |
| `refBaseHeight` | Empty: read nominal height from controller YAML |
| `paramsFile` | Installed `config/dualsense.yaml` |
| `deviceName` | `DualSense Wireless Controller`; takes precedence over `deviceId` |
| `deviceId` | `0`; clear `deviceName` to select by ID |
| `startJoy` | `true`; set `false` if a driver is already publishing `joyTopic` |
| `joyTopic` | `/joy` |
| `commandTopic` | `/humanoid/walking_velocity_command` |
| `use_sim_time` | `false`; watchdog and publishing use wall/steady time either way |

Output velocities are normalized fractions within `[-1, 1]`, not physical m/s or rad/s.
`desired_pelvis_height` is an absolute height in meters, matching the GUI contract.
The bundled driver uses `<joyTopic>/raw`; the adapter subscribes with best effort and exposes the unchanged joystick data on public `joyTopic` with best effort, volatile, depth 1.
Commands and `~/status` diagnostics also use best effort, volatile, depth 1.
With `startJoy:=false`, the adapter directly consumes the external topic using best effort; that external publisher's QoS is not changed.
Only one base-command source may publish at once; disable the GUI and scripted targets before using teleop.
Axis/button mapping and safety parameters are read-only after startup; restart to apply a new mapping.
Optional motion-axis and height-button indices can be `-1` to disable them; the enable axis and stop button are always required.


## Tests

```bash
colcon test --packages-select legged_robot_teleop --event-handlers console_direct+
colcon test-result --verbose
```

C++ unit tests (gtest, `test/test_control.cpp`) cover signs/scaling, height bounds, release/stop, malformed input, held commands without new input, the stick response curve, and the enable FSM: post-Center re-enabling, trigger hysteresis, blocked-input recovery, clock faults, and 6,000 deterministic randomized input steps.
The ROS-level pytest tests start the built `base_twist` executable and observe it only through topics, `~/status` and the ROS graph.
Isolated service tests cover Square edges, configurable mapping, blocked presses, transition states, stale robot status and L2 re-arming at handover.
The ROS transport test uses its own ROS domain and unique test topics, including a paused simulation clock with no `/clock` publisher.
It never sends robot commands or starts a simulation.
