# Handoff for Claude Code (started in a Claude desktop session)

ROS 1 Noetic (Ubuntu 20.04 VM) project for the Yahboom ROSMASTER X3 Plus (CSIS-370 Robotics).
Read README.md first: it documents the shape-drawing package (base drives shapes, simulated with
`fake_odom_sim.py`; build/run steps, parameters and safety caps are there). Do not duplicate it here.

## Two separate things in this repo
1. **Base shape drawing** (existing, `src/`, `shapes/`, `launch/`): mecanum base follows shape YAMLs. Works in sim
   via `launch/draw_shape_sim.launch` (use `source sim_env.sh` for a local ROS master).
2. **Arm drawing** (new, UNTESTED, `arm/arm_draw.py`): MoveIt Cartesian paths of preset shapes
   (circle, square, triangle, star, heart, line, zigzag) in the arm tip's y-z plane, group `arm_group`,
   speed/accel scaling 0.2, size `S = 0.03` m. Run only in simulation first:
   `roslaunch arm_moveit_demo x3plus_moveit_demo.launch sim:=true`, then `python3 arm/arm_draw.py [shape ...]`.
   It prints the reachable fraction per shape and skips shapes below 95%.
   Verify the group name and that the script compiles before trusting it.

## Worksheet context (course assignment)
- Map the lab with gmapping and save as `roboticsLabMap` (.yaml + .pgm) into ~/yahboomcar_ws/src/yahboomcar_nav/maps/.
  Robot side: `roslaunch yahboomcar_nav laser_bringup.launch`, then
  `roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping`
  (Raspberry Pi 5 robots: run ~/run_docker.sh first, same container in every terminal).
  VM side: `roslaunch yahboomcar_nav view_map.launch`; teleop with `rosrun beginner_tutorials yahboomcar_keyboard.py`
  (move 0.14, turn 0.4). Save: `roscd yahboomcar_nav/maps/ && rosrun map_server map_saver -f roboticsLabMap`.
- Virtual arm: install `ros-noetic-trac-ik*`, then the MoveIt demo above.
- Yahboom docs: https://www.yahboom.net/study/ROSMASTER-X3-PLUS/8176 (mapping), /6029 (arm MoveIt).

## Rules
- Never run anything that moves the real robot or arm without asking the user first; sim only by default.
- The user types passwords. Robot IP/username are not recorded here.
- Repo is public: never commit passwords, IPs, keys or personal data.
- Nothing from the arm work or the mapping has been run yet.
