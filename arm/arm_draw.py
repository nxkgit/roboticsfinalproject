#!/usr/bin/env python3
# Draws preset shapes with the X3 Plus arm tip in a vertical (y-z) plane using MoveIt.
# Run AFTER: roslaunch arm_moveit_demo x3plus_moveit_demo.launch sim:=true
#
# Usage:
#   python3 arm_draw.py                 # draws every shape in turn
#   python3 arm_draw.py square star     # draws only the ones you name
# Shapes: circle square triangle star heart line zigzag
import sys, math, copy
import rospy, moveit_commander

S = 0.03  # shape size in metres (half-width). Start small!


def circle(n=36):
    return [(S * math.sin(2 * math.pi * i / n), S * (math.cos(2 * math.pi * i / n) - 1))
            for i in range(n + 1)]


def square():
    return [(0, 0), (S, 0), (S, S), (-S, S), (-S, -S), (S, -S), (S, 0), (0, 0)]


def triangle():
    return [(0, 0), (S, 0), (0, S * 1.6), (-S, 0), (0, 0)]


def star():
    pts = []
    for k in range(0, 11):  # 5-point star, points on a circle
        a = math.pi / 2 + k * 4 * math.pi / 5
        pts.append((S * math.cos(a), S * (math.sin(a) - 1)))
    return [(0, 0)] + pts[:6] + [(0, 0)]


def heart(n=40):
    pts = []
    for i in range(n + 1):
        t = 2 * math.pi * i / n
        y = 16 * math.sin(t) ** 3
        z = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)
        pts.append((y * S / 17.0, (z - 5) * S / 17.0))  # scaled; starts/ends at the top dip
    return [(0, 0)] + pts + [(0, 0)]


def line():
    return [(0, 0), (S * 2, 0), (0, 0)]


def zigzag():
    return [(0, 0), (S, S), (S * 2, 0), (S * 3, S), (S * 4, 0), (0, 0)]


SHAPES = {"circle": circle, "square": square, "triangle": triangle,
          "star": star, "heart": heart, "line": line, "zigzag": zigzag}


def draw(arm, name, start):
    waypoints = []
    for dy, dz in SHAPES[name]():
        p = copy.deepcopy(start)
        p.position.y = start.position.y + dy
        p.position.z = start.position.z + dz
        waypoints.append(p)
    plan, fraction = arm.compute_cartesian_path(waypoints, 0.005, 0.0)
    print("%-9s reachable: %3.0f%%" % (name, fraction * 100))
    if fraction > 0.95:
        arm.execute(plan, wait=True)
        rospy.sleep(1.0)
    else:
        print("  skipped: not enough of it is reachable. Try a smaller S or a different start pose.")


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arm_draw")
    arm = moveit_commander.MoveGroupCommander("arm_group")  # group name from Yahboom page
    arm.set_max_velocity_scaling_factor(0.2)       # slow and safe
    arm.set_max_acceleration_scaling_factor(0.2)

    names = [a for a in rospy.myargv(argv=sys.argv)[1:] if a in SHAPES] or list(SHAPES)
    start = arm.get_current_pose().pose            # every shape starts/ends here
    for n in names:
        draw(arm, n, start)
    print("done")
