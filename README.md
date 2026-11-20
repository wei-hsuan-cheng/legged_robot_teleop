# Legged Robot Teleop

DualSense gamepad commands for legged robot MPC, using ROS 2 Jazzy's standard `joy` driver.
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
| Circle | Stop velocity commands; release L2 before enabling again |
| Release L2 or lose input | Zero velocity/yaw, keep the selected height |

The launch YAML currently uses 100% of the MPC's configured maximum speeds; start with small stick deflections.
Lower the fractions in `config/dualsense.yaml` for slower joystick control without changing MPC tuning.
Upkie's robot profile disables independent sideways commands.
Input loss longer than 0.25 wall seconds disarms motion; reconnection with L2 held cannot restart it.
L2 is axis 4, reported by `game_controller_node` as 0 released to -1 fully pressed.
The deadman engages at 50% of that ROS range and releases at 20%, avoiding threshold chatter.
Centered sticks and released Center/height buttons are required at the instant of enabling.
Center requires L2 at or below its release threshold and a fresh L1 press.
L1 presses during driving or partial trigger presses are ignored, not queued for later.
Release L1 before re-enabling L2.
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

Use `robotName:=g1` or `robotName:=upkie` for the other profiles.
The launch file reads `defaultBaseHeight`, `minBaseHeight`, and `maxBaseHeight` directly from the selected robot's controller YAML.
If robot launch uses `controllersFile` or `refBaseHeight`, pass the same values to teleop.
No robot height settings are duplicated in this package.


## Configuration

Edit [config/dualsense.yaml](config/dualsense.yaml) or provide a copy with `paramsFile:=/absolute/path.yaml`.
The driver is `game_controller_node`, not `joy_node`; it provides standardized SDL button/axis ordering and already uses positive forward/left ROS signs.
See the [upstream driver documentation](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md).

| Argument | Default / Purpose |
| --- | --- |
| `robotName` | `g1`; also `h2`, `upkie` |
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
Output QoS is best effort, volatile, depth 1; input accepts best-effort or reliable Joy publishers.
Only one base-command source may publish at once; disable the GUI and scripted targets before using teleop.
Axis/button mapping and safety parameters are read-only after startup; restart to apply a new mapping.
Optional motion-axis and height-button indices can be `-1` to disable them; the enable axis and stop button are always required.


## Tests

```bash
colcon test --packages-select legged_robot_teleop --event-handlers console_direct+
colcon test-result --verbose
```

Unit tests cover signs/scaling, height bounds, release/stop, malformed input, timeouts, and reconnection.
The ROS transport test uses its own ROS domain and unique test topics, including a paused simulation clock with no `/clock` publisher.
It never sends robot commands or starts a simulation.
