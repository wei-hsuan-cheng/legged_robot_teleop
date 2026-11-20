# Teleop Handoff


## Scope

New standalone ROS 2 Jazzy package `legged_robot_teleop` in `/Users/whcheng/src/mpc/legged_robot_teleop`, created at the user's request for a PS5 DualSense.
The user authorized a public `wei-hsuan-cheng/legged_robot_teleop` repository and a VCS entry in `legged_robot_mpc_controller`.
The adapter uses standard `joy/game_controller_node`, not a custom hardware driver.
No robot MPC model, solver, estimator, scene, or gains are changed.
At the user's subsequent request, `robot.launch.py` uses one string selector, `baseCommandDevice:=gui|joystick|none`, default `gui`.
`joystick` includes this package's launch; `gui` starts the GUI; `none` leaves input to scripted or external commands.
The old GUI boolean is retired, and GUI/teleop cannot coexist through the launch selector.
It forwards the already-resolved controller file and nominal height, avoiding duplicate robot settings.
The MPC manifest, runtime dependency, and README get the new optional-input-package integration.


## Observations

Ubuntu detected Sony USB ID `054c:0ce6` at `/dev/input/event11` and `/dev/input/js1`.
The existing privileged `ros_jazzy` container lacked the new input nodes because it started before the controller was connected and does not bind-mount `/dev/input`.
The user restarted it and confirmed raw ROS input works.
Enumeration now reports a mapped gamepad named `DualSense Wireless Controller`, device ID 0.
The upstream SDL game-controller ROS driver already flips SDL axes to ROS-positive forward/left; no additional sign reversal is appropriate by default.
The GUI and MPC use normalized velocity/yaw fields and absolute height in meters, not a `geometry_msgs/Twist` topic.


## Safety Design

L1 is a deadman and requires release then press with centered enabled axes and released height controls.
Circle clears permission to move; releasing L1 or a 0.25-second input timeout also zeros velocity while preserving height.
Malformed Joy arrays, nonfinite values, invalid button values, and out-of-range axes disarm.
Reconnect with L1 held cannot auto-enable.
The timer uses a steady clock and the watchdog uses monotonic receipt time, independent of ROS simulation time.
Default speed fractions are 0.25 in each direction; existing MPC command conditioning remains unchanged.
Profile launch reads the selected controller YAML for height bounds and disables lateral input when its physical limit is zero, including Upkie.
This is not a hardware E-stop, does not stop MPC balancing, and cannot protect against teleop process death or downstream transport failure.
No automatic arbitration is provided; only one base-command publisher may run.


## Validation

The package built successfully in `ros_jazzy`, symlinked from `/root/ocs2_ros2_ws/src/legged_robot_teleop` to `/root/src/mpc/legged_robot_teleop`.
All 39 package checks passed, including 29 command-control cases, seven profile cases, an isolated real-DDS transport test with paused simulation time, and two Python lint checks.
The MPC launch selector passed all three modes without launching nodes.
After moving the checkout into `mpc/`, the package was rebuilt to refresh its Python development links.
The MPC Release rebuild also passed (42.5 seconds); existing signedness warnings remain in its native model test.
Installed robot launch argument discovery confirms `baseCommandDevice` and the teleop include.
All 39 package checks passed again after relocation, with no warnings after running Python lint before the DDS test.
The three input-selection checks plus seven MPC package-health checks passed (10 total).
No live-device launch, robot simulation, or production command publishing is performed during this build-only checkpoint.
Robot driving, cable-unplug response with a live gamepad, and Bluetooth need user acceptance after the topic checks.
Do not start a robot or publish to `/humanoid/walking_velocity_command` during isolated checks.


## Next Milestones

1. Let the user verify actual stick signs, height controls, release, unplug, and reconnect on the isolated topic.
2. User-controlled flat-ground simulation with `baseCommandDevice:=joystick`.
3. Optional persistent Docker input hotplug configuration, without disrupting current robot jobs.
4. Optional direct Ubuntu Bluetooth pairing and repeated disconnect acceptance.
