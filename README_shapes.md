# Shape drawing for the Yahboom ROSMASTER X3 Plus (ROS 1 Noetic)

Drives the mecanum base along a predefined shape (square, rectangle, triangle,
circle, star, figure-8). The shape is scaled and centered so the whole path fits
inside a W × H m area after subtracting a safety margin and the robot footprint;
shapes that do not fit are **refused** with the numbers. The robot translates in
x/y and holds its start heading — corners are changes of direction, never turns
in place. The "drawing" is the recorded odometry path, shown in RViz and saved as
a PNG of planned vs. actual path plus a CSV.

## Files

| File | Purpose |
|---|---|
| `src/shape_geometry.py` | Pure Python (stdlib + yaml): load, densify, fit-check, plan, frames, controller math, metrics |
| `src/test_shape_geometry.py` | `unittest` tests for the above |
| `src/turtle_square_openloop.py` | Timed open-loop square (`_mode:=turn` for turtlesim, `_mode:=strafe` for mecanum) |
| `src/fake_odom_sim.py` | Simulator: integrates `/cmd_vel` into `nav_msgs/Odometry` on `/odom` |
| `src/shape_drawer.py` | Executor: plans the shape and follows it on odometry |
| `src/path_recorder.py` | Records `~actual_path`, writes CSV + PNG + metrics to `~/shape_runs` |
| `shapes/*.yaml` | Shape specs (normalized coordinates; only proportions matter) |
| `launch/draw_shape_sim.launch` | Simulation: fake_odom_sim + shape_drawer + path_recorder (+ RViz) |
| `launch/draw_shape.launch` | Real robot: shape_drawer + path_recorder (bringup runs on the robot) |
| `rviz/shapes.rviz` | Fixed frame `odom`; planned (blue) and actual (orange) paths |
| `sim_env.sh` | Points ROS at a **local** master for simulation |

Build: `cd ~/catkin_ws && catkin_make && chmod +x src/beginner_tutorials/src/*.py`
(added dependency: `nav_msgs`).

## Frames and fitting

Area frame A: origin at the area's lower-left corner, +x along W, +y along H.
The robot starts at (`start_x`, `start_y`) in A — default the area center —
facing +x. The robot center may use the **usable box**
`(m+r, m+r, W−m−r, H−m−r)`; with the defaults (W = H = 1.5, m = r = 0.15) that is
0.30–1.20 m on both axes (0.9 × 0.9 m).

The shape's bounding box (arc extremes included) is scaled uniformly and centered
in the usable box. `size:=0` uses the largest scale that fits; `size:=S` makes the
longest side S m. The plan is **refused** (exit code 1, no `/cmd_vel` publisher)
if the usable box is empty, `size` is larger than the box, the fitted shape is
smaller than 0.20 m, or the start is outside the usable box. A shape is never
shrunk silently.

## Shape format

```yaml
name: square
start: [0.0, 0.0]                  # first point; robot transits here from its start pose
segments:
  - {type: line, to: [1.0, 0.0]}
  - {type: arc, center: [0.5, 0.5], sweep_deg: -90}   # radius = |current point - center|; + = CCW
```
+x is the robot's forward direction at the start and +y its left. Every segment end
is a stop point; arcs are split into chords ≤ `max_chord` m (pass points).

## shape_drawer parameters

| Param | Default | Meaning |
|---|---|---|
| `~shape` | `square` | name in `shapes/` or a path to a YAML file |
| `~shapes_dir` | package `shapes/` | |
| `~area_w`, `~area_h` | 1.5, 1.5 | area size (m) |
| `~margin` | 0.15 | safety margin inside the area (m) |
| `~footprint_r` | 0.15 | robot radius: half the chassis diagonal (m) — **measure it** |
| `~size` | 0.0 | 0 = largest fit, else longest side in m |
| `~start_x`, `~start_y` | area center | robot start in the area frame (m) |
| `~max_chord` | 0.05 | max arc chord (m) |
| `~speed` | 0.2 | m/s; hard-capped at **0.30** in code |
| `~turn` | 1.0 | rad/s; hard-capped at **1.0** in code |
| `~min_speed` | 0.03 | deadband floor while outside tolerance (m/s) |
| `~kp`, `~kyaw` | 1.5, 2.0 | position / heading P gains |
| `~stop_tol`, `~pass_tol` | 0.02, 0.04 | waypoint tolerances (m) for stop / pass points |
| `~rate` | 20 | control rate (Hz) |
| `~odom_timeout` | 0.5 | abort if odometry is older than this (s) |
| `~odom_topic` | `/odom` | **verify on the robot** |
| `~cmd_vel_topic` | `/cmd_vel` | |
| `~dry_run` | false | print the plan, exit 0, never create a `/cmd_vel` publisher |
| `~require_confirm` | true | on a non-local master, type `YES` before moving |
| `~wait_for_recorder` | 3.0 | wait up to this long (s) for path_recorder before moving |

Topics: `~planned_path` (`nav_msgs/Path`, latched, odom frame), `~status`
(`std_msgs/String`: `running k/N`, `done`, `aborted: <reason>`).

Aborts (zero Twist, status, exit code 2): stale odometry, pose outside the area
minus `footprint_r` (geofence), a waypoint taking longer than `3·dist/speed + 5` s,
rospy shutdown / Ctrl-C. A zero Twist is also published 3× in `finally`.

`fake_odom_sim` params: `~rate` 50, `~linear_scale` 1.0, `~angular_scale` 1.0,
`~noise_std` 0.0 (m per step while moving), `~cmd_timeout` 0.5, `~x0 ~y0 ~yaw0` 0.
`path_recorder` params: `~odom_topic` `/odom`, `~out_dir` `~/shape_runs`, `~max_poses` 5000.

## Simulation (local master only)

Every command starts with `source ~/catkin_ws/src/beginner_tutorials/sim_env.sh &&`
— your normal shell points at the robot. Check it echoes
`ROS_MASTER_URI=http://localhost:11311`.

```bash
# unit tests
cd ~/catkin_ws/src/beginner_tutorials/src && python3 -m unittest -v test_shape_geometry
# plan only
source ~/catkin_ws/src/beginner_tutorials/sim_env.sh && roslaunch beginner_tutorials draw_shape_sim.launch shape:=star dry_run:=true
# run (add rviz:=true to watch)
source ~/catkin_ws/src/beginner_tutorials/sim_env.sh && roslaunch beginner_tutorials draw_shape_sim.launch shape:=figure8
# wheel-calibration error + noise: feedback still closes the shape
source ~/catkin_ws/src/beginner_tutorials/sim_env.sh && roslaunch beginner_tutorials draw_shape_sim.launch shape:=square linear_scale:=1.05 noise_std:=0.002
# turtlesim open-loop square (three terminals, each starting with the sim_env.sh source)
source ~/catkin_ws/src/beginner_tutorials/sim_env.sh && roscore
source ~/catkin_ws/src/beginner_tutorials/sim_env.sh && rosrun turtlesim turtlesim_node
source ~/catkin_ws/src/beginner_tutorials/sim_env.sh && rosrun beginner_tutorials turtle_square_openloop.py _mode:=turn
```
Results land in `~/shape_runs/<YYYYmmdd_HHMMSS>_<shape>.{csv,png}`.

Note: closure and corner error are bounded by `stop_tol` (2 cm default): the robot
counts a corner as reached within 2 cm and heads for the next one. Use
`_stop_tol:=0.01` for tighter corners (slower settling).

## Verified simulation results (fake_odom_sim, defaults, 2026-11-04)

| Shape | Status | Time (s) | Closure (cm) | Cross-track max / mean (cm) |
|---|---|---|---|---|
| square | done | 25.6 | 1.85 | 1.89 / 0.74 |
| rectangle | done | 20.5 | 1.97 | 1.81 / 0.60 |
| triangle | done | 20.0 | 1.88 | 1.58 / 0.60 |
| circle | done | 33.5 | 1.96 | 2.48 / 0.42 |
| star | done | 29.0 | 1.93 | 1.77 / 0.51 |
| figure8 | done | 30.7 | 1.83 | 0.82 / 0.65 |
| square, `linear_scale:=1.05 noise_std:=0.002` | done | 24.5 | 1.92 | 3.41 / 0.85 |

Also verified: dry run and `size:=1.0` refusal create no `/cmd_vel` publisher
(polled the master every 20 ms); refusal exits 1; Ctrl-C, odometry loss and a
geofence breach abort with exit 2 and a zero Twist; the `YES` gate works through
`roslaunch` stdin. These are **odometry-frame** numbers from a simulator whose
odometry is perfect; they say nothing about accuracy on the real floor.

## Real-robot checklist (do these in order; stop at any failure)

The drawer never moves the real robot without you typing `YES` in its terminal.
Keep one person on **Ctrl-C** and one on the robot's **power switch** for every
moving step.

1. **Link check.** Robot powered, `roslaunch yahboomcar_bringup bringup.launch`
   running on it. Open a **normal terminal (do NOT source `sim_env.sh`)** and repeat
   the `~/robot_teleop.sh` checks by hand:
   ```bash
   echo $ROS_MASTER_URI $ROS_IP          # http://192.168.2.113:11311 and a 192.168.2.x address
                                          # 10.0.2.x = VirtualBox NAT -> switch Adapter 1 to Bridged, reboot
   ping -c 1 -W 2 192.168.2.113
   timeout 3 bash -c "</dev/tcp/192.168.2.113/11311" && echo master up
   rostopic info /cmd_vel                 # Subscribers: must list the robot's base driver
   ```
2. **Resolve the unknowns** (read-only commands):
   - Odometry topic/type:
     `rostopic list | grep -i odom`, then `rostopic info <topic>`, then
     `rostopic echo -n 1 <topic>`. Expect `/odom` or `/odom_raw`, type
     `nav_msgs/Odometry`. Pass it as `odom_topic:=<topic>`. If the type is not
     `nav_msgs/Odometry`, stop — the drawer needs that type.
   - Heading: `rostopic list | grep -i imu`. With `rostopic echo <odom topic>/pose/pose/orientation`
     running, rotate the robot ~90° by hand; yaw = `atan2(2(w·z + x·y), 1 − 2(y² + z²))`
     should change by ~1.57 rad and come back. If it does not, heading hold
     cannot work — stop.
   - Velocity limits and dead-reckoning accuracy on your floor: unknown until
     steps 5–7; keep `speed:=0.1`.
3. **Footprint.** Tape-measure the chassis including wheels and any arm overhang
   (park the arm). `footprint_r` = half the diagonal of that rectangle. Cross-check
   against the URDF (read only, do not edit):
   `find ~/yahboomcar_ws -iname "*x3*urdf*"`. Pass `footprint_r:=<value>`.
4. **Dry run** (no `/cmd_vel` publisher is created):
   `roslaunch beginner_tutorials draw_shape.launch shape:=square dry_run:=true odom_topic:=<topic> footprint_r:=<r>`
   Check the usable box, scale and waypoint table make sense for your area.
5. **Wheels off the ground.** Put the robot on a stand so no wheel touches.
   `roslaunch beginner_tutorials draw_shape.launch shape:=square speed:=0.1 odom_topic:=<topic> footprint_r:=<r>`
   Read the plan, type `YES`. Wheels should spin in the expected pattern (forward /
   strafe) and stop at the end; Ctrl-C must stop them immediately. Note: odometry
   still integrates while lifted, so a geofence/timeout abort here is acceptable.
6. **On the floor.** Tape out the W × H area and mark the start pose (center,
   facing +x = along W). Clear the area.
   `roslaunch beginner_tutorials draw_shape.launch shape:=square size:=0.5 speed:=0.1 odom_topic:=<topic> footprint_r:=<r> area_w:=<W> area_h:=<H>`
   Type `YES` only when the Ctrl-C and power-switch people are ready.
7. **Measure.** Tape-measure the closure error (start mark to final position)
   and compare it with the closure error in the log / the PNG in `~/shape_runs`.
   The PNG shows *odometry*, so a gap between PNG and tape is dead-reckoning error.
