#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-10-28
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Shape Drawing (Milestone 3)
#
# shape_drawer.py
# Drives the mecanum base along a shape from shapes/*.yaml. The shape is
# scaled and centered inside the usable part of a W x H area (or refused),
# then followed waypoint by waypoint with a P controller on odometry,
# translating in x/y while holding the start heading (no turns in place).
#
# Safety: hard caps 0.30 m/s and 1.0 rad/s, odometry watchdog, geofence,
# per-waypoint timeout, operator YES gate on a non-local master, dry run
# never creates a /cmd_vel publisher, zero Twist on every exit path.
#
# Exit codes: 0 done / dry run, 1 refused or bad input, 2 aborted.
# ---------------------------------------------------------
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

import shape_geometry as sg

# Hard limits, applied regardless of ~speed / ~turn
MAX_LINEAR = 0.30    # m/s
MAX_ANGULAR = 1.0    # rad/s
SETTLE_TIME = 0.3    # s of zero command at each stop point
ODOM_WAIT = 5.0      # s to wait for the first odometry message
LINGER_TIME = 1.5    # s to keep publishing zero after the run so the recorder sees the final status

DEFAULT_SHAPES_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'shapes'))


class AbortRun(Exception):
    """Raised to stop the run; the message becomes the 'aborted: <reason>' status."""


def make_twist(vx, vy, wz, linear_cap, angular_cap):
    """Build a Twist, clamping |(vx, vy)| to linear_cap (direction kept) and |wz| to angular_cap."""
    v = math.hypot(vx, vy)
    if v > linear_cap:
        vx, vy = vx * linear_cap / v, vy * linear_cap / v
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = max(-angular_cap, min(angular_cap, wz))
    return twist


def needs_confirmation(master_uri, require_confirm):
    """True when the master is not local and the operator must type YES."""
    local = 'localhost' in master_uri or '127.0.0.1' in master_uri
    return require_confirm and not local


def waypoint_timeout(dist, speed):
    """Seconds allowed to reach a waypoint dist meters away."""
    return 3.0 * dist / speed + 5.0


def plan_summary(p, fit, spec, wps, start_xy, cap_v):
    """Multi-line text describing the plan (used for dry run and the YES gate)."""
    bx = sg.spec_bbox(sg.transform_spec(spec, fit.scale, fit.offset))
    ub = sg.usable_box(p['area_w'], p['area_h'], p['margin'], p['footprint_r'])
    length = sg.path_length(wps, start_xy)
    lines = [
        'shape        : %s' % spec.get('name'),
        'area         : %.2f x %.2f m, margin %.2f m, footprint_r %.2f m'
        % (p['area_w'], p['area_h'], p['margin'], p['footprint_r']),
        'usable box   : x %.3f..%.3f, y %.3f..%.3f m (%.3f x %.3f m)'
        % (ub[0], ub[2], ub[1], ub[3], ub[2] - ub[0], ub[3] - ub[1]),
        'scale        : %.4f m per shape unit' % fit.scale,
        'shape bbox   : x %.3f..%.3f, y %.3f..%.3f m (%.3f x %.3f m)'
        % (bx[0], bx[2], bx[1], bx[3], bx[2] - bx[0], bx[3] - bx[1]),
        'robot start  : (%.3f, %.3f) m in area frame, facing +x' % start_xy,
        'waypoints    : %d (%d stop points)' % (len(wps), sum(1 for w in wps if w.stop)),
        'path length  : %.3f m (incl. %.3f m transit from start)'
        % (length, math.hypot(wps[0].x - start_xy[0], wps[0].y - start_xy[1])),
        'speed cap    : %.2f m/s -> estimated time %.1f s' % (cap_v, sg.estimate_time(wps, start_xy, cap_v, SETTLE_TIME)),
    ]
    return '\n'.join(lines)


def waypoint_table(wps):
    rows = ['   k        x        y  stop']
    for k, w in enumerate(wps):
        rows.append('%4d  %7.3f  %7.3f  %s' % (k, w.x, w.y, 'STOP' if w.stop else 'pass'))
    return '\n'.join(rows)


def read_params():
    p = {
        'shape': rospy.get_param('~shape', 'square'),
        'shapes_dir': rospy.get_param('~shapes_dir', DEFAULT_SHAPES_DIR),
        'area_w': float(rospy.get_param('~area_w', 1.5)),
        'area_h': float(rospy.get_param('~area_h', 1.5)),
        'margin': float(rospy.get_param('~margin', 0.15)),
        'footprint_r': float(rospy.get_param('~footprint_r', 0.15)),
        'size': float(rospy.get_param('~size', 0.0)),
        'max_chord': float(rospy.get_param('~max_chord', 0.05)),
        'speed': float(rospy.get_param('~speed', 0.2)),
        'turn': float(rospy.get_param('~turn', 1.0)),
        'min_speed': float(rospy.get_param('~min_speed', 0.03)),
        'kp': float(rospy.get_param('~kp', 1.5)),
        'kyaw': float(rospy.get_param('~kyaw', 2.0)),
        'stop_tol': float(rospy.get_param('~stop_tol', 0.02)),
        'pass_tol': float(rospy.get_param('~pass_tol', 0.04)),
        'rate': float(rospy.get_param('~rate', 20.0)),
        'odom_timeout': float(rospy.get_param('~odom_timeout', 0.5)),
        'odom_topic': rospy.get_param('~odom_topic', '/odom'),
        'cmd_vel_topic': rospy.get_param('~cmd_vel_topic', '/cmd_vel'),
        'dry_run': bool(rospy.get_param('~dry_run', False)),
        'require_confirm': bool(rospy.get_param('~require_confirm', True)),
        'wait_for_recorder': float(rospy.get_param('~wait_for_recorder', 3.0)),
    }
    # Start defaults to the area center; launch files pass -1 for "unset"
    sx = float(rospy.get_param('~start_x', -1.0))
    sy = float(rospy.get_param('~start_y', -1.0))
    p['start_x'] = sx if sx >= 0 else p['area_w'] / 2.0
    p['start_y'] = sy if sy >= 0 else p['area_h'] / 2.0
    # Publish the resolved values so path_recorder can draw the boxes
    for key in ('area_w', 'area_h', 'margin', 'footprint_r', 'start_x', 'start_y', 'stop_tol'):
        rospy.set_param('~' + key, p[key])
    return p


class OdomTracker(object):
    """Latest odometry pose plus the local time it arrived (robot clock may differ)."""

    def __init__(self, topic):
        self.pose = None
        self.frame_id = 'odom'
        self.received = None
        rospy.Subscriber(topic, Odometry, self.on_odom, queue_size=1)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     sg.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.frame_id = msg.header.frame_id or 'odom'
        self.received = time.time()

    def age(self):
        return float('inf') if self.received is None else time.time() - self.received


def safe_publish(pub, msg):
    """Publish, ignoring errors from a publisher that rospy already closed during shutdown."""
    try:
        pub.publish(msg)
    except rospy.ROSException:
        pass


def make_path(points, frame_id):
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = frame_id
    for pt in points:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = pt[0]
        ps.pose.position.y = pt[1]
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def follow(p, wps_odom, odom, odom0, start_xy, cmd_pub, status_pub, cap_v, cap_w):
    """Run the control loop; returns normally when done, raises AbortRun otherwise."""
    rate = rospy.Rate(p['rate'])
    r = p['footprint_r']
    fence = (r, r, p['area_w'] - r, p['area_h'] - r)
    yaw0 = odom0[2]
    n = len(wps_odom)
    k = 0
    status = None
    wp_start = time.time()
    x, y, _ = odom.pose
    wp_limit = waypoint_timeout(math.hypot(wps_odom[0].x - x, wps_odom[0].y - y), cap_v)

    while True:
        if rospy.is_shutdown():
            raise AbortRun('rospy shutdown')
        if odom.age() > p['odom_timeout']:
            raise AbortRun('odometry timeout (%.2f s since last message > %.2f s)'
                           % (odom.age(), p['odom_timeout']))
        x, y, yaw = odom.pose
        ax, ay = sg.odom_to_area(x, y, start_xy, odom0)
        if not sg.inside_box(ax, ay, fence):
            raise AbortRun('geofence: pose (%.3f, %.3f) m left area x %.2f..%.2f, y %.2f..%.2f m'
                           % (ax, ay, fence[0], fence[2], fence[1], fence[3]))

        wp = wps_odom[k]
        tol = p['stop_tol'] if wp.stop else p['pass_tol']
        ex, ey = wp.x - x, wp.y - y
        if math.hypot(ex, ey) <= tol:
            if wp.stop:
                settle_end = time.time() + SETTLE_TIME
                while time.time() < settle_end and not rospy.is_shutdown():
                    cmd_pub.publish(Twist())
                    rate.sleep()
            k += 1
            if k == n:
                return
            x, y, _ = odom.pose
            wp_start = time.time()
            wp_limit = waypoint_timeout(math.hypot(wps_odom[k].x - x, wps_odom[k].y - y), cap_v)
            continue

        if time.time() - wp_start > wp_limit:
            raise AbortRun('waypoint %d/%d not reached after %.1f s (limit %.1f s), %.3f m away'
                           % (k + 1, n, time.time() - wp_start, wp_limit, math.hypot(ex, ey)))

        new_status = 'running %d/%d' % (k + 1, n)
        if new_status != status:
            status = new_status
            status_pub.publish(String(status))

        vx, vy = sg.body_velocity(ex, ey, yaw, p['kp'], cap_v, p['min_speed'], tol)
        wz = sg.heading_rate(yaw0, yaw, p['kyaw'], cap_w)
        cmd_pub.publish(make_twist(vx, vy, wz, cap_v, cap_w))
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            raise AbortRun('rospy shutdown')


def main():
    rospy.init_node('shape_drawer')
    p = read_params()
    cap_v = min(p['speed'], MAX_LINEAR)
    cap_w = min(p['turn'], MAX_ANGULAR)
    start_xy = (p['start_x'], p['start_y'])

    # 1. Load and plan; refuse before any publisher exists
    try:
        spec = sg.load_shape(p['shape'], p['shapes_dir'])
    except (ValueError, IOError) as e:
        rospy.logerr('shape_drawer: %s', e)
        sys.exit(1)
    fit = sg.fit_shape(spec, p['area_w'], p['area_h'], p['margin'], p['footprint_r'],
                       size=p['size'], start=start_xy)
    if not fit.ok:
        rospy.logerr('shape_drawer: REFUSED %s: %s', spec.get('name'), fit.reason)
        sys.exit(1)
    if cap_v <= 0 or cap_w <= 0:
        rospy.logerr('shape_drawer: ~speed and ~turn must be > 0 (got %.3f, %.3f)', p['speed'], p['turn'])
        sys.exit(1)
    wps = sg.densify(sg.transform_spec(spec, fit.scale, fit.offset), p['max_chord'])
    summary = plan_summary(p, fit, spec, wps, start_xy, cap_v)

    # 2. Dry run: print and leave without a /cmd_vel publisher
    if p['dry_run']:
        print('\n=== DRY RUN (no /cmd_vel publisher) ===\n%s\n%s\n' % (summary, waypoint_table(wps)))
        sys.exit(0)

    # 3. First odometry, then the plan in the odom frame
    odom = OdomTracker(p['odom_topic'])
    deadline = time.time() + ODOM_WAIT
    while odom.pose is None and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)
    if odom.pose is None:
        rospy.logerr('shape_drawer: no odometry on %s within %.0f s, not moving', p['odom_topic'], ODOM_WAIT)
        sys.exit(2)
    odom0 = odom.pose
    rospy.set_param('~odom0', list(odom0))
    wps_odom = sg.area_to_odom(wps, start_xy, odom0)
    plan_pub = rospy.Publisher('~planned_path', Path, queue_size=1, latch=True)
    plan_pub.publish(make_path(wps_odom, odom.frame_id))
    status_pub = rospy.Publisher('~status', String, queue_size=10, latch=True)

    # 4. Operator gate on a real (non-local) master
    master_uri = os.environ.get('ROS_MASTER_URI', '')
    if needs_confirmation(master_uri, p['require_confirm']):
        print('\n=== PLAN on %s ===\n%s\nodom0       : (%.3f, %.3f, %.1f deg)\n'
              % (master_uri, summary, odom0[0], odom0[1], math.degrees(odom0[2])))
        sys.stdout.flush()
        try:
            answer = input('Robot will MOVE. Type YES to start: ')
        except EOFError:
            answer = ''
        if answer.strip() != 'YES':
            rospy.logwarn('shape_drawer: not confirmed (%r), exiting without moving', answer.strip())
            status_pub.publish(String('aborted: not confirmed'))
            time.sleep(0.5)
            sys.exit(2)

    # Give path_recorder (slow to start: matplotlib) time to subscribe so the drawing starts at the start
    deadline = time.time() + p['wait_for_recorder']
    while status_pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)

    # 5. Control loop
    cmd_pub = rospy.Publisher(p['cmd_vel_topic'], Twist, queue_size=1)
    rospy.on_shutdown(lambda: safe_publish(cmd_pub, Twist()))
    rospy.loginfo('shape_drawer: %s, %d waypoints, speed cap %.2f m/s, odom0 (%.3f, %.3f, %.1f deg)',
                  spec.get('name'), len(wps), cap_v, odom0[0], odom0[1], math.degrees(odom0[2]))
    exit_code = 0
    t0 = time.time()
    try:
        follow(p, wps_odom, odom, odom0, start_xy, cmd_pub, status_pub, cap_v, cap_w)
        cmd_pub.publish(Twist())
        status_pub.publish(String('done'))
        rospy.loginfo('shape_drawer: done in %.1f s', time.time() - t0)
    except (AbortRun, KeyboardInterrupt) as e:
        reason = str(e) or 'rospy shutdown (Ctrl-C)'
        safe_publish(cmd_pub, Twist())
        safe_publish(status_pub, String('aborted: %s' % reason))
        rospy.logerr('shape_drawer: aborted: %s', reason)
        exit_code = 2
    finally:
        # Always leave the robot stopped (time.sleep: rospy.sleep raises during shutdown)
        for _ in range(3):
            safe_publish(cmd_pub, Twist())
            time.sleep(0.05)
    # Hold zero briefly so path_recorder receives the final status before roslaunch tears down
    end = time.time() + LINGER_TIME
    while time.time() < end and not rospy.is_shutdown():
        cmd_pub.publish(Twist())
        time.sleep(0.1)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
