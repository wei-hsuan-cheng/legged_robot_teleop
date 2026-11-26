# Teleop Handoff


## Scope

New standalone ROS 2 Jazzy package `legged_robot_teleop` in `/Users/whcheng/src/mpc/legged_robot_teleop`, created at the user's request for a PS5 DualSense.
The user authorized a public `wei-hsuan-cheng/legged_robot_teleop` repository and a VCS entry in `legged_robot_mpc_controller`.
The adapter uses standard `joy/game_controller_node`, not a custom hardware driver.
No robot MPC model, solver, estimator, scene, or gains are changed.
At the user's subsequent request, `robot.launch.py` uses one string selector, `base_command_device:=gui|joystick|none`, default `gui`.
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

L2 is the deadman and requires release then press with centered enabled axes and released height controls.
The user changed it from the initial L1 mapping before their robot acceptance test.
Standard `game_controller_node` reports L2 as axis 4, 0 released to -1 pressed, so `sign_enable` is -1.
Engage at pressure 0.5 and disengage/re-arm at 0.2, with hysteresis and no need for an L1 button mapping.
Circle clears permission to move; releasing L2 or a 0.25-second input timeout also zeros velocity while preserving height.
Malformed Joy arrays, nonfinite values, invalid button values, and out-of-range axes disarm.
Reconnect with L2 held cannot auto-enable.
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
Installed robot launch argument discovery confirms `base_command_device` and the teleop include.
All 39 package checks passed again after relocation, with no warnings after running Python lint before the DDS test.
The three input-selection checks plus seven MPC package-health checks passed (10 total).
No live-device launch, robot simulation, or production command publishing is performed during this build-only checkpoint.
L2-specific regression coverage includes gradual presses, hysteresis, timeout/stop re-arming, missing trigger axes, and L1 not enabling motion.
After the L2 change, the package rebuilt and all 48 checks passed without warnings (38 control, seven profile, one DDS transport, two lint).
The user requested pushing this change and the pending MPC launch/VCS/docs integration to their respective branches.


## Center Command

L1 now means Center, replacing the enabled-mode Triangle height reset; its mapping parameter is `button_center`, default 9.
Only a fresh L1 press with L2 at or below the release threshold can reset height to nominal and keep all velocity/yaw fields zero.
L1 during driving or a partial trigger press is ignored and never queued until L2 release.
Holding L1 blocks arming; Circle has priority over Center.
The same existing publisher sends the complete command, with no second publisher or mixed active/center modes.
MPC command conditioning remains responsible for the physical reference transition; Center is not a hardware stop.
The user's YAML changes (100 Hz publishing, full speed scales, 0.2 m/s height rate), now committed separately as `2785fbf`, are preserved.
Teleop rebuilt successfully with L1 Center, and all 52 checks passed, including the extended DDS transition check.
No robot simulation or production command publishing was started; restart the existing teleop launch to load the new mapping.
Robot driving, cable-unplug response with a live gamepad, and Bluetooth need user acceptance after the topic checks.
Do not start a robot or publish to `/humanoid/walking_velocity_command` during isolated checks.


## Next Milestones

1. Let the user verify actual stick signs, height controls, release, unplug, and reconnect on the isolated topic.
2. User-controlled flat-ground simulation with `base_command_device:=joystick`.
3. Optional persistent Docker input hotplug configuration, without disrupting current robot jobs.
4. Optional direct Ubuntu Bluetooth pairing and repeated disconnect acceptance.


## Explicit Enable FSM and Joystick QoS

The user reported intermittent L2 enabling and restricted this work to `legged_robot_teleop`; the MPC repository is not modified.
Two new regression tests reproduced an old implementation failure before the fix: Center followed by L1 release and L2 press in the same input snapshot, or L1 release during a gradual L2 press, both failed to enable despite neutral controls.
The implicit `enabled`/`_released` combination is replaced by one `EnableState`: `WAIT_RELEASE`, `READY`, or `ENABLED`.
L2 release qualifies READY even with Center held, while the actual enable press still requires L1/D-pad released and neutral axes.
Unsafe enable attempts, stop, malformed input, stale input, and invalid time require a fresh L2 release; clearing a blocked control while keeping L2 held cannot auto-enable.
L2 thresholds, the 0.25-second timeout, 100 Hz command publishing, full speed fractions, and 0.2 m/s height rate are unchanged.
Blocked reasons remain latched until a valid release and are logged with the phase and trigger pressure.
Best-effort `~/status` publishes a JSON diagnostic on reason changes and once per second.
The standard driver now outputs to `<joy_topic>/raw`; the same adapter node subscribes best effort at depth 1 and republishes public `joy_topic` best effort at depth 1, without modifying the driver or adding a process.
The standard driver's raw publisher retains its original QoS; external drivers used with `start_joy:=false` keep their own publisher QoS.
Tests cover direct and bridged real DDS paths, public publisher reliability, local depth-1 profiles, no alias-induced relay loop, and 6,000 randomized safety-invariant steps.
Some DDS discovery implementations report history depth as zero, so tests check reliability through discovery and depth through the actual rclpy publisher profile.
See [FSM documentation](docs/enable_fsm.md) for exact transition and recovery rules.
The package rebuilt successfully and all 78 checks passed before the user requested no further testing.
The two post-Center regressions failed on the prior implementation and passed on the explicit FSM.
No robot was started, no production commands were published, and the MPC repository remained unchanged.
Further testing stopped at the user's request; restart teleop to load the changes.


## Upkie Operator Posture Requests (2026-11-21)

The user requested real Stand Up / Sit Down controls in both GUI and joystick, with DualSense R1 as the toggle.
The adapter reads `bringUp.standUpTrajectoryFile` from the same controller YAML as its height profile and sets `posture_controller` only when that trajectory exists.
R1 (SDL button 10, configurable as `button_posture`) calls that controller's `~/toggle_stand` Trigger service on a fresh edge.
It requires released L2, neutral sticks and released L1/D-pad/Circle; held-at-connect, blocked, repeated and intermediate-state presses are never queued.
Only the controller chooses whether a valid toggle stands or sits.
Best-effort `~/motion_state` feedback must be fresh within 0.5 wall seconds; driving is disarmed outside locomotion and on stale state.
The operator must release and press L2 again after stand-up.
The existing trigger FSM, Center semantics, numerical speed/height settings and joystick QoS remain unchanged.
All 80 checks passed, including real isolated DDS posture requests and the older 6,000-step trigger regressions.
The MPC integration has also completed estimated-state repeated stand/sit cycles and a deliberate latched-fall trial; its final test results and artifacts are recorded in that repository's wheeled-biped handoff.
No physical controller or hardware acceptance is claimed.
Held R1 after a joystick input gap is also consumed until release; this is checked with real DDS input.
Robot state and Joy arrive independently, so the controller remains the final authority for accepting a posture request even when client state is a heartbeat behind.
The final 80-check run passed without errors or skipped tests, and both packages' final install refresh passed.
The user then requested committing and pushing this package on `main` together with the MPC integration on `feat/wheeled_biped`.


## Square Posture Toggle (2026-11-21)

The user replaced the R1 stand/sit toggle with DualSense Square.
The node default and shipped YAML now use `button_posture: 2` (the standard game_controller_node SDL X / Square index).
R1 no longer triggers a posture request by default; a deliberate `button_posture` override remains supported.
L2, L1 Center, Circle, motion axes, height controls and all posture/trigger interlocks are unchanged.
The isolated service regression runs against both the default Square mapping and an explicit R1 override, checks that the other button does nothing, and retains blocked/held/reconnect/transition coverage.
A separate check confirms the shipped YAML specifies Square.
Both packages built, and all 82 teleop checks passed without skips.
The companion MPC guides and synthetic-Joy cycle harness use Square; previous R1 simulation results remain historical evidence rather than being relabeled.
Two estimated-state Square-driven stand/drive/sit cycles passed at a 200 Hz MPC target, with maximum locomotion-labelled tilt 0.07047 rad.
The simulation was stopped after acceptance; artifacts are `/tmp/codex_upkie_square.*` in `ros_jazzy`.
This validates synthetic Joy through the real teleop service path, not physical gamepad or hardware behavior.
The user requested committing and pushing both repositories after testing.


## Circle Unassigned (2026-11-22)

The user requested removing Circle's current function and reserving it for future use.
The stop-button setting, YAML binding, stop handling and posture-button blocker were removed rather than remapped.
Circle remains present in the unchanged raw/public Joy messages but has no action in the adapter.
It cannot stop or arm driving, block L1 Center, or request/block Square stand/sit.
L2 release, input timeout, malformed-input guards, robot-state guards and all other bindings remain in effect.
Earlier Circle-stop statements above describe historical checkpoints, not the current behavior.
Regression tests cover ignored Circle while arming/driving, Center with Circle held, and real DDS Square requests with Circle held.
The shared package built in both the OCS2 and TurboMPC workspaces; each passed all 82 checks without errors, failures or skips.
No robot launch was started or stopped for these isolated checks.
The user requested pushing the shared-package change and synchronized operator guides.
