# Shape-Drawing Robot — Early Design Review (Script)

*Framing: this is written as if delivered right after M2 (the timed open-loop
square) — M3/M4 (closed-loop shape drawer, simulator, recorder) are described
as the plan, not as finished work. Swap in your own name/course header.*

---

## 1. Problem & Motivation

"My final project is to get a Yahboom ROSMASTER X3 Plus — a mecanum-wheeled
base running ROS 1 Noetic — to physically trace out a 2D shape on the floor:
square, rectangle, triangle, circle, star, figure-eight. The 'drawing' isn't
ink, it's the robot's own path — we prove it drew the shape by recording its
odometry and comparing the planned path against the actual one.

The reason mecanum matters: a mecanum base can translate in any direction
without turning in place. So the plan is for the robot to hold a single
heading the whole time and change *direction*, not *orientation*, at every
corner. That's a deliberate constraint I'm choosing, not something forced on
me — it's what makes this an interesting control problem instead of just
'turtle graphics with turns.'"

## 2. Goals & current status

"Three things have to be true for me to call this done:

1. Shapes are defined once, in a robot-agnostic format, and automatically
   scaled to fit whatever physical area I give it — not hand-tuned per shape.
2. The robot follows that shape using feedback — odometry in, velocity
   commands out — not a fixed timed sequence.
3. I can prove how well it worked, numerically, not just by eyeballing it.

Where I am right now: **milestones 1 and 2 are done.** Milestone 1 is a pure
Python geometry layer — no ROS dependency at all — that loads a shape
spec, fits it into a usable area, turns arcs into short line segments, and
has a full unit test suite. Milestone 2 is a timed, open-loop square: no
feedback, just 'drive forward for X seconds, strafe for Y seconds,' proven out
first on turtlesim and then with hard velocity caps for the real base. That
was deliberately the easy version first, to de-risk the ROS plumbing —
publishers, params, topics — before I add a feedback loop on top of it.

**Milestones 3 and 4 are the plan I want to walk through today**, since
that's the actual control and validation work."

## 3. Planned system architecture

"Here's the node graph I'm planning for M3/M4. Everything after milestone 2
is built on three ROS nodes plus one ROS-independent library:

- **`shape_geometry` (library, no ROS).** Loads a YAML shape spec, fits and
  centers it inside a configurable W×H area (accounting for a safety margin
  and the robot's footprint radius), densifies arcs into short chords, and
  holds the frame-transform and P-controller math. It's a plain Python module
  specifically so I can unit-test the geometry and control math without ever
  starting ROS.
- **`shape_drawer` (node — the executor).** On startup it plans the shape
  using `shape_geometry`, publishes that plan once as a latched
  `nav_msgs/Path` so it's visible in RViz, then runs a control loop:
  subscribe to `/odom`, compute position and heading error to the current
  waypoint, run it through a P controller, publish a `geometry_msgs/Twist` on
  `/cmd_vel`. When it's within tolerance of a waypoint it advances to the
  next one. It also publishes a `/status` string (`running k/N`, `done`,
  `aborted: <reason>`).
- **`fake_odom_sim` (node — simulation only).** Subscribes to `/cmd_vel`,
  integrates it forward into a `nav_msgs/Odometry` message on `/odom`. This
  is the stand-in for the real robot's odometry while I'm developing on a
  laptop with no base attached — it even supports injected wheel-calibration
  bias and Gaussian noise, so I can test whether the feedback loop actually
  corrects for imperfect odometry before I ever touch the real robot.
- **`path_recorder` (node).** Subscribes to `/odom`, records the actual path,
  and on completion writes a CSV, a PNG overlay of planned (blue) vs. actual
  (orange) path, and the error metrics below.

On the real robot, `fake_odom_sim` is simply not launched — the robot's own
`yahboomcar_bringup` stack provides real odometry and the real motor driver,
and `shape_drawer` / `path_recorder` don't know the difference.

Two separate launch files are planned for this: one that brings up
`fake_odom_sim` + `shape_drawer` + `path_recorder` (+ RViz) for simulation,
and one that only brings up `shape_drawer` + `path_recorder` for the real
robot, since bringup runs separately on the robot itself."

## 4. Shape representation

"Shapes are specified in a normalized, robot-agnostic coordinate frame — a
start point plus a list of line and arc segments, in a YAML file. Only the
*proportions* matter; `shape_geometry` figures out the scale and offset
needed to center that shape inside whatever physical area and robot footprint
I tell it about at runtime. If a shape genuinely doesn't fit — too big for the
area, or smaller than a sane minimum after scaling — the plan is for the node
to refuse outright and print the numbers, rather than silently drawing a
shrunk or distorted version of it."

## 5. Control approach

"M2 proved the ROS plumbing with an open loop. M3 is where I add the actual
feedback: at every control tick, position error becomes a body-frame
velocity command (P gain, clamped to a max speed, raised to a minimum speed
while still outside tolerance so it doesn't stall in the motor deadband), and
heading error becomes a yaw-rate command to hold the start heading. Waypoints
are marked as either 'stop' points (segment ends — settle before continuing)
or 'pass' points (interior arc points — just get close and keep moving)."

## 6. Safety plan

"Since this ends with a real robot moving around a room, the plan has several
hard stops built in from the start, not bolted on later:
- A geofence — abort if the robot's pose leaves the usable area.
- An odometry staleness check — abort if `/odom` goes quiet.
- A per-waypoint time budget — abort if a leg is taking far longer than its
  expected distance/speed would predict.
- Hard velocity caps in code that apply whenever the topic isn't a
  turtlesim topic, regardless of what parameter is passed in.
- A typed `YES` confirmation gate before any motion on a non-local ROS
  master — i.e. before it ever moves the real base.
Every abort path, and `finally`, publishes a zero `Twist` before exiting."

## 7. Validation plan

"The validation plan is staged specifically so that the first time the real
robot moves is as late and as controlled as possible:
1. Unit tests on `shape_geometry` — no ROS, pure math, run on every change.
2. Simulated closed-loop runs against `fake_odom_sim`, including runs with
   deliberately injected wheel-calibration bias and odometry noise, to check
   the feedback loop actually closes the shape despite imperfect odometry.
3. A dry-run mode that prints the full plan and exits without ever creating a
   `/cmd_vel` publisher — so I can sanity-check scaling/fit on the real
   robot's parameters with zero risk of motion.
4. On the real robot: a link/topic-discovery check, a footprint measurement,
   wheels-off-the-ground test, then a slow floor test — in that order, with
   someone on Ctrl-C and someone on the power switch for every step that can
   move.
Success is measured two ways: *closure error* (distance from the robot's
final position back to its start) and *cross-track error* (distance from
each actual point to the nearest point on the planned path) — not just 'it
looked like a square.'"

## 8. Open risks / what I don't know yet

"Being honest about what's unresolved at this stage:
- I don't yet know how much the real robot's dead-reckoning drifts over a
  multi-meter shape — simulated odometry is perfect by construction, so the
  simulation results won't tell me that; only the real-floor test will.
- I haven't confirmed the exact odometry topic name/type on the physical
  robot yet, or measured its footprint radius.
- Mecanum strafe calibration (x vs. y gain mismatch) is a known failure mode
  for this kind of base, and I don't know yet how big it is on this unit.
- Timeline: M3 (closed-loop node + simulator) and M4 (recording, RViz,
  real-robot checklist) are both still ahead of me."

## 9. Status / next steps

"Here's the order I'm planning to actually do this in — software fully
validated in simulation before the real robot is involved at all:

**Software (M3)**
1. Write `fake_odom_sim` first — it's the smaller piece, and nothing else
   can be tested without it.
2. Write `shape_drawer`'s control loop against it: the planning/latched-path
   publish, the `/odom` subscription, the P controller for position and
   heading, waypoint advancement, and the abort paths (geofence, odometry
   staleness, per-waypoint time budget).
3. Add the `dry_run` mode and the `YES` confirmation gate before wiring up
   the real `/cmd_vel` publisher — these are cheap to add early and remove a
   whole class of 'oops it moved' mistakes later.

**Recording & visualization (M4)**
4. Write `path_recorder`: subscribe to `/odom`, buffer the actual path,
   write the CSV + closure/cross-track metrics + the planned-vs-actual PNG
   on completion.
5. Write the two launch files (sim: `fake_odom_sim` + `shape_drawer` +
   `path_recorder` [+ RViz]; real: `shape_drawer` + `path_recorder` only)
   and an RViz config showing planned (blue) vs. actual (orange) path.

**Simulated validation — still before the real robot**
6. Run all six shapes through the sim at default settings; confirm each one
   reports `done`, not an abort, and record closure/cross-track error per
   shape.
7. Re-run with deliberately injected wheel-calibration bias and odometry
   noise (`fake_odom_sim`'s `linear_scale` / `noise_std` params) to check the
   feedback loop actually corrects for imperfect odometry rather than just
   working because the simulator is too clean.
8. Confirm the negative cases on purpose: `dry_run` and an oversized `size:=`
   request both report a refusal and create *no* `/cmd_vel` publisher
   (checked by polling the ROS master); Ctrl-C, odometry loss, and a
   geofence breach all abort with a zero `Twist`.

**Real robot (M4, staged — stop at any failure)**
9. Link check: robot powered and reachable, `rostopic info /cmd_vel` shows
   the robot's own driver as a subscriber.
10. Discover the real odometry topic/type and confirm heading actually
    tracks a hand rotation of the robot — stop here if it doesn't, since
    heading hold can't work without it.
11. Measure the footprint radius by hand and cross-check it against the
    base's URDF.
12. `dry_run` on the real robot's parameters — sanity-check the usable box
    and waypoint table before anything can move.
13. Wheels off the ground, low speed — confirm the motion pattern and that
    Ctrl-C stops it immediately.
14. On the floor, low speed, small shape, with someone on Ctrl-C and someone
    on the power switch.
15. Tape-measure the real closure error and compare it against the
    recorded/plotted odometry closure error — the gap between the two *is*
    the real robot's dead-reckoning error, which is the number I don't have
    yet.

That last step is really the point of the whole real-robot phase: simulated
closure error only tells me the control loop is correct, not how accurate
the real robot's odometry is. Step 15 is where I'd actually find that out."

---

## Anticipated questions

**Q: What is the system architecture?**
A: Three ROS nodes plus one ROS-independent library. `shape_geometry` does
all the math (loading, fitting/scaling, densifying, frame transforms,
controller math) with no ROS dependency, so it's unit-testable standalone.
`shape_drawer` is the executor — it plans once, then closes a feedback loop
between `/odom` (in) and `/cmd_vel` (out). `fake_odom_sim` stands in for the
real robot's odometry during simulation by integrating `/cmd_vel` forward.
`path_recorder` subscribes to `/odom` and produces the CSV/PNG/metrics that
prove how well it worked. Two launch files assemble these differently for
sim vs. the real robot.

**Q: Why mecanum / why not just turn at the corners like a differential
robot?**
A: Turning in place introduces its own odometry error (wheel-slip during
rotation is typically worse than during translation) and it's a less
interesting control problem. Holding one heading and changing direction
instead is what the mecanum base is actually good for, and it means every
corner is a translation-error problem, which is the thing I'm trying to
measure and correct.

**Q: How do you know the robot actually drew the right shape — what's your
evidence?**
A: Two numbers per run: closure error (does it end up back where it started)
and cross-track error (how far off the planned line was it at every recorded
point), plus a plotted overlay of planned vs. actual path. Not a subjective
"it looked right."

**Q: What happens if the robot drifts outside the safe area, or something
goes wrong mid-shape?**
A: It's designed to abort, not coast — a geofence check, an odometry
staleness timeout, and a per-waypoint time budget all trigger an abort that
publishes a zero velocity command immediately, and the same zero-command
publish happens in a `finally` block no matter how the node exits.

**Q: How will you validate this before ever running it on the real robot?**
A: Unit tests on the pure-math layer first, then closed-loop simulation
against a fake odometry node — including deliberately injected calibration
bias and noise — and only after that a staged real-robot sequence (link
check, parameter discovery, footprint measurement, dry run, wheels-off-ground,
then a slow floor test), each with an explicit abort criterion before
proceeding to the next.

**Q: What shapes does it support, and how are they defined?**
A: Square, rectangle, triangle, circle, star, and figure-eight right now,
each as a small YAML file: a start point and a sequence of line/arc segments
in normalized coordinates. Adding a shape just means adding a YAML file — it
doesn't touch the control code.

**Q: Is this hard-coded to one room size or one robot?**
A: No — the area size, safety margin, robot footprint radius, start position
within the area, and desired shape size are all runtime parameters. If a
requested configuration genuinely can't fit, the plan is to refuse with the
specific numbers rather than silently shrinking or distorting the shape.

**Q: Why build the open-loop square first instead of going straight to
closed-loop?**
A: To de-risk the ROS plumbing — publishers, parameters, topic wiring —
separately from the control math. If something had gone wrong with the
open-loop version, I'd have known it was plumbing, not the controller.

**Q: What's your fallback if the real robot's odometry turns out to be too
unreliable to use?**
A: That's genuinely an open risk I don't have a resolved answer for yet —
it's one of the things listed as unknown until the real-floor test. If
dead-reckoning drift turns out to dominate, the honest fallback is reporting
that limitation rather than overstating accuracy the system doesn't have.

**Q: Why ROS 1 Noetic instead of ROS 2?**
A: That's what the Yahboom ROSMASTER X3 Plus's vendor stack (bringup,
drivers) targets, so it's the practical choice for this hardware rather than
a deliberate preference between the two.

**Q: For people who don't know ROS — what's a `Twist` / `/cmd_vel`?**
A: `/cmd_vel` is the standard topic a ROS robot listens to for velocity
commands; a `Twist` message is just "move at this x/y speed and rotate at
this rate." Every node that wants the robot to move publishes one of these;
my `shape_drawer` node is the only thing computing what that command should
be at each instant.
