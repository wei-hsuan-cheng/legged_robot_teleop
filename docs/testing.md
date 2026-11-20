# Connection and Testing


## 1. USB to Ubuntu

Connect the controller with a data-capable USB-C cable.
In Parallels, attach the controller to Ubuntu through Devices > USB, not to macOS.
Run on Ubuntu:

```bash
lsusb
ls -l /dev/input/by-id/
```

Expect a Sony DualSense entry and an `event-joystick` symlink.
A charging light alone is not a successful data connection.
See [Parallels USB passthrough](https://kb.parallels.com/en/122993).


## 2. Docker Access

Run these checks outside and inside the container; the controller's input event node must exist in both.

```bash
# Ubuntu VM
ls -l /dev/input/by-id/*event-joystick
docker exec ros_jazzy ls -l /dev/input
```

The ROS `joy` driver uses `/dev/input/event*`, not only `/dev/input/js*`.
A privileged container started before plugging in the controller may lack the newly created nodes.
If no robot jobs are running, restart the container after connecting the controller:

```bash
docker restart ros_jazzy
docker exec -it ros_jazzy bash
```

Restarting stops every process in the container, including a running simulation.
For persistent hotplug support, a future container configuration can bind-mount `/dev/input:/dev/input:ro` and permit access to the input devices.
The existing privileged container already has device permission, but currently does not bind-mount this directory.
Changing container configuration requires recreating it; preserve its workspace/build data before doing so.
No container configuration change is required for the working wired setup.


## 3. Inspect Raw Input

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_enumerate_devices
ros2 run joy game_controller_node --ros-args \
  -p device_name:="DualSense Wireless Controller" \
  -p deadzone:=0.08 -p autorepeat_rate:=50.0
```

In another sourced terminal, run `ros2 topic echo /joy`.
Check each stick and button, then Ctrl+C both commands before launching the combined teleop launch.
The [driver mapping](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md) assigns left/right sticks to axes `[0, 1]` and `[2, 3]`, L2 to axis 4, Circle to button 1, Triangle to 3, and D-pad up/down to 11/12.
Check that L2 changes from 0 released toward -1 fully pressed; L1 is not the deadman.
Use the actual enumerated device name if it differs over Bluetooth.


## 4. Isolated Command Check

The following command does not publish to the MPC command topic and does not launch a robot:

```bash
source /root/ocs2_ros2_ws/install/setup.bash
ros2 launch legged_robot_teleop teleop.launch.py \
  robotName:=h2 \
  joyTopic:=/teleop/test/joy \
  commandTopic:=/teleop/test/walking_velocity_command
```

In another sourced terminal:

```bash
ros2 topic echo /teleop/test/walking_velocity_command \
  --qos-reliability best_effort
```

Release L2, center sticks, then hold L2 and move them.
Enable occurs at 50% of the ROS trigger range; release to 20% or below to disable and re-arm.
Full forward produces `linear_velocity_x: 0.25`; full left produces `linear_velocity_y: 0.25`; right stick left produces positive yaw.
H2 starts at the nominal height from its installed YAML; D-pad changes it only while L2 is held.
Release L2 and check zero velocity/yaw with unchanged height.
Press Circle while holding L2 and check that motion cannot resume until L2 is released and pressed again with centered sticks.
Unplug the cable while commanding motion: output should become zero within approximately 0.25 seconds plus one publish interval, subject to host scheduling.
Reconnect with L2 held and check that it stays disabled until released and enabled again.
This timeout covers loss of Joy input while teleop is alive, not a killed process or DDS/network failure downstream.
Do not rely on one final best-effort shutdown packet as an emergency stop.

If a driver is already running, use `startJoy:=false` and its actual `joyTopic` instead of starting a duplicate driver.


## 5. Robot Simulation

Stop the isolated teleop launch before this step.
Ensure the robot scene is flat for the first test and the target mode is `base_twist`.
Do not change MPC gains or gait parameters for this test.
For the integrated launch, stop any standalone gamepad driver and teleop first, then run:

```bash
ros2 launch legged_robot_mpc_controller robot.launch.py \
  robotName:=h2 mpcFreq:=80 mrtFreq:=1000 \
  baseCommandDevice:=joystick rviz:=true mujoco_headless:=false
```

This starts the gamepad driver and adapter, suppresses the GUI, and shares the robot's height settings automatically.
Wait for stable standing before enabling L2.
For independent teleop launch, start the robot without either input frontend:

```bash
ros2 launch legged_robot_mpc_controller robot.launch.py \
  robotName:=h2 mpcFreq:=80 mrtFreq:=1000 \
  baseCommandDevice:=none rviz:=true mujoco_headless:=false
```

After initialization and stable standing, run in a second sourced terminal:

```bash
ros2 launch legged_robot_teleop teleop.launch.py robotName:=h2
```

Test small forward/backward motions first, then yaw, sideways motion, and small height changes.
Release L2 between each test.
Repeat the release and disconnect checks in simulation.
For G1 use `robotName:=g1` in both launches; for Upkie use `robotName:=upkie`, `mpcFreq:=200`, and wait for its stand-up sequence before enabling teleop.
Upkie supports forward/yaw/height but not independent sideways drive.
The 25% speed scales cap H2 forward command at 0.5 m/s with the current 2.0 m/s MPC maximum; G1/Upkie use their own current maxima.
Never run the GUI, figure-eight targets, and gamepad output on the same robot command topic simultaneously.


## 6. Bluetooth

Only proceed after the wired input and disconnect tests pass.
The direct-to-Ubuntu route uses a Linux-compatible USB Bluetooth adapter passed through to the VM.
Disconnect the controller's USB cable, hold Create + PS until its light bar flashes, and pair it in Ubuntu Bluetooth settings.
See [Sony pairing instructions](https://www.playstation.com/en-us/support/hardware/pair-dualsense-controller-bluetooth/).
Parallels' documented macOS virtual-gamepad sharing targets Windows; do not assume it exposes a controller to Linux.
See [Parallels gamepad support](https://kb.parallels.com/en/124771).
Enumerate devices again, update `deviceName` if necessary, and repeat the isolated topic and connection-loss checks before driving.
