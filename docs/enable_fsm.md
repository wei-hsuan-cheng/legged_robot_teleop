# Enable State Machine


## Permission States

Only `ENABLED` can produce nonzero velocity/yaw or integrate height, roll and pitch changes.
The public `enabled` property is derived from this state, rather than maintained as another flag.
The command topic, robot limits, speed fractions, height rate, and L2 thresholds are unchanged.

| State | Meaning | Transition |
| --- | --- | --- |
| `wait_release` | Startup, fault, stop, or unsafe enable attempt | A fresh valid input with L2 pressure <= 0.2 enters `ready` |
| `ready` | L2 release has been observed; output velocity/yaw is zero | Pressure >= 0.5 with centered motion axes, L1 released, and D-pad released enters `enabled` |
| `enabled` | Hold-to-drive | L2 pressure <= 0.2 returns to `ready`; a fault returns to `wait_release` |

The pressure band between 0.2 and 0.5 preserves the previous permission state.
There is no release debounce delay; a released L2 immediately clears motion commands.
An unsafe enable attempt consumes that press, so simply centering a stick while L2 remains held cannot unexpectedly start motion.
Release L2 and retry when the reported blocking input is clear.
Malformed input and nonfinite/backward receipt or timer time also clear permission. Silent input does not: there is no input timeout.
All non-enabled states clear cached velocity, height-stick and D-pad inputs, preserving the selected height and tilt targets.
The right stick vertical axis must be inside `height_axis_threshold` (default 0.3) before arming or toggling stand/sit.
Height stays fixed inside that deadband, so small vertical offsets during yaw do not change height or block enabling.
Above the threshold, its rate scales continuously to `height_rate` at full stick.
An absent or malformed configured height axis clears permission; `axis_height: -1` disables height input.
Pitch/roll D-pad buttons block arming and stand/sit only if that robot has a nonzero limit for the corresponding tilt axis.


## Center Interaction

L1 Center is accepted only on a fresh button edge with L2 at or below its release threshold.
It restores nominal height, levels roll/pitch and leaves velocity/yaw zero through the existing command publishers.
L1 presses while driving or partially pressing L2 are consumed without queueing a later reset.
Circle is unassigned and does not affect motion permission or Center.
Holding L1 blocks enabling, but does not erase the fact that L2 was physically released.
Therefore, a subsequent message may safely release L1 and press L2 together, provided the other controls are neutral.
This fixes the earlier requirement for an extra input message with both L1 and L2 released.


## Diagnostics

The node logs phase changes and blocked-enable reasons, with the last valid L2 pressure.
The best-effort `~/status` topic also publishes JSON on reason changes and once per wall second:

```bash
ros2 topic echo /base_twist_teleop/status --qos-reliability best_effort
```

Fields are `phase`, `reason`, `enabled`, and `last_trigger_pressure`.
In `wait_release`, the last fault or unsafe-press reason is retained until a valid trigger release, rather than overwritten by repeated held-trigger messages.
The trigger pressure is the last valid input value, not a claim that a disconnected controller remains held.

| Reason | Action |
| --- | --- |
| `waiting_for_input` | Check driver, device selection, and joystick topic |
| `release_trigger_to_enable` | Release L2, then press it with neutral controls |
| `enable_blocked_sticks_not_centered` | Release L2, center the sticks, then press L2 again |
| `enable_blocked_center_held` | Release L2 and L1 before trying again |
| `enable_blocked_pitch_buttons_held` | Release L2 and D-pad up/down before trying again |
| `enable_blocked_roll_buttons_held` | Release L2 and D-pad left/right before trying again |
| `invalid_input` | Check driver mapping and message shape |
| `invalid_time` | Check the receipt/timer time source, then release L2 and retry |

In `ready`, `center_held`, `pitch_buttons_held`, `roll_buttons_held`, and `sticks_not_centered` are preparation hints rather than faults.
`ready` means neutral controls and L2 below the engage threshold; `centered` records an accepted Center edge.


## Joystick QoS

The bundled standard driver publishes to `<joy_topic>/raw` using its original publisher QoS.
The teleop node subscribes to that stream with best effort, volatile, depth 1, processes it, and republishes the unchanged Joy message on `joy_topic` with best effort, volatile, depth 1.
Thus the default public `/joy` topic is best effort without modifying the standard driver or adding another process.
The relay does not subscribe to its own output, including when absolute and relative names resolve to the same topic.
Base commands and status also use best effort with depth 1.
Depth 1 retains the latest sample rather than replaying a queue of older stick commands.
With `start_joy:=false`, the adapter subscribes directly to the external `joy_topic` using best effort and does not create another publisher there.
An external driver's publisher QoS remains that driver's responsibility.
Publishing and watchdog checks use steady/monotonic time even when simulation time is paused.
These protections are not a hardware emergency stop and do not cover adapter process death or downstream command-transport loss.
