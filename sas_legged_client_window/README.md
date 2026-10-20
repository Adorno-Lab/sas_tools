# sas_legged_client_window

Qt window to monitor or command a legged robot driver of [sas_tools](..) (e.g. the Unitree G1 or B1), as a controller
would: it only uses `LeggedRobotDriverClient` for the base and one `RobotDriverClient` per limb. The limbs, the
supported modes, and the limits of the base are read from `get/info`, so the same window works with any robot that
follows the standard.

It shows the status, the high-level mode, the IMU, the measured twist (if the robot publishes it), and the joint
states of every limb. It has the mode buttons,
sliders for the twist and for the base pose (orientation and height), sliders for the limbs that accept targets, an
emergency stop, and a watchdog.

Requires Qt 6 (`sudo apt install qt6-base-dev`).

## Usage

```shell
ros2 launch sas_legged_client_window sas_legged_client_window_launch.py legged_robot_prefix:=sas_b1/b1_1
```

or

```shell
ros2 run sas_legged_client_window sas_legged_client_window --ros-args -p legged_robot_prefix:=sas_g1/g1_1
```

| Parameter | Default | Description |
|---|---|---|
| `legged_robot_prefix` | `sas_b1/b1_1` | Topic prefix of the legged robot driver |
| `own_driver_limb_prefixes` | `[]` | Limbs with their own driver, e.g. `['sas_z1/z1_1']` for the Z1 arm of a B1 |
| `mode` | `control` | `control`: the window commands the robot. `monitor`: it only reads the robot (the command topics are not created), so that it can run next to the controller |
| `max_vx`, `max_vy`, `max_wz` | `0.2`, `0.1`, `0.2` | Ranges of the twist sliders, in m/s and rad/s (conservative, for testing) |
| `watchdog_period_in_seconds` | `1.0` | Initial period of the watchdog, in the period slider (1 to 5 s, rounded) |
| `watchdog_maximum_acceptable_delay_in_seconds` | `0.5` | Maximum acceptable delay of the triggers. The driver fixes it at the first trigger. Across computers whose clocks are not synchronized, use a large value |
| `shutdown_topic` | `/sas/set/shutdown` | The emergency stop also publishes on this topic: the one of the emergency stop of the lab, to which the launch files of the drivers remap their `set/shutdown`. Empty string to disable |

The launch file has the same arguments, except the ranges of the twist sliders, plus `node_name`: to run several
windows (e.g. one per robot), give each one its own name.

## Behaviour

> [!CAUTION]
> In control mode, the robot moves with the sliders. Be ready to perform an emergency stop!

- The mode buttons show the mode reported by the robot; the modes that it does not support are disabled.
- The twist is sent every 50 ms while the robot accepts it (`WALKING`): if the window stops sending it, the driver
  stops the robot after its twist timeout. The twist and base pose sliders go back to zero (the neutral pose) on every
  mode request, and whenever the robot does not accept them.
- The base pose (`STANDING`) is sent when a slider moves. The ranges of the orientation sliders are the limits of
  `get/info`, minus one step (0.001 rad), since at the limit itself the conversion to a quaternion and back can exceed
  it slightly. The range of the height slider is `[min_base_height, max_base_height]`, and its neutral position is
  `nominal_base_height`.
- The limbs of the legged driver accept targets when the driver says so (`get/status`). A limb with its own driver
  does not know the mode of the base, so it only accepts targets while its *Allow commands* box is checked. While a
  limb does not accept targets, its sliders follow the measured positions, so that the first drag starts from there.
- `EMERGENCY STOP` sends the shutdown signal to the legged driver and to every limb with its own driver, and on
  `shutdown_topic` (`/sas/set/shutdown`), which stops every driver whose `set/shutdown` is remapped there (e.g. the B1
  and the Z1), on the same `ROS_DOMAIN_ID`.
- The period of the watchdog is chosen with its slider, from 1 to 5 s, before it starts: the driver fixes it at the
  first trigger, so the slider is disabled once the watchdog runs.
- `START WATCHDOG` (after a confirmation) sends watchdog triggers to the legged driver and to every limb with its own
  driver, ten per period, from their own thread. **It cannot be stopped**: when the window closes (or crashes), the
  triggers stop, and the drivers stop the robot. To watch a controller, run a second window in monitor mode and start
  its watchdog there: only one client per driver may send the triggers.
- Closing the window (or `Ctrl + C`) sends a zero twist.
