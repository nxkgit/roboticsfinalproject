#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-10-28
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Shape Drawing (Milestone 4)
#
# path_recorder.py
# Records the odometry path while shape_drawer runs ("the drawing").
# Publishes ~actual_path (nav_msgs/Path, capped), and on done / aborted /
# shutdown writes <out_dir>/<YYYYmmdd_HHMMSS>_<shape>.csv (t, x, y, yaw)
# and a .png of planned (dashed) vs. actual (solid) in the area frame,
# then logs closure and cross-track error in cm.
# ---------------------------------------------------------
import csv
import os
import threading
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt   # noqa: E402

import rospy                                         # noqa: E402
from geometry_msgs.msg import PoseStamped            # noqa: E402
from nav_msgs.msg import Odometry, Path              # noqa: E402
from std_msgs.msg import String                      # noqa: E402

import shape_geometry as sg                          # noqa: E402

DRAWER_NS = '/shape_drawer'
PUBLISH_PERIOD = 0.2      # s between ~actual_path updates

# Reference palette slots 1 and 2; identity is also carried by line style + legend
COLOR_PLANNED = '#2a78d6'
COLOR_ACTUAL = '#eb6834'
COLOR_BOX = '#52514e'


class PathRecorder(object):

    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.out_dir = os.path.expanduser(rospy.get_param('~out_dir', '~/shape_runs'))
        self.max_poses = int(rospy.get_param('~max_poses', 5000))
        self.lock = threading.Lock()
        self.samples = []          # (t, x, y, yaw) in odom
        self.planned = []          # (x, y) in odom
        self.frame_id = 'odom'
        self.status = ''
        self.final = None          # 'done' / 'aborted: ...' once seen
        self.written = False

        self.path_pub = rospy.Publisher('~actual_path', Path, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber(DRAWER_NS + '/planned_path', Path, self.on_plan, queue_size=1)
        rospy.Subscriber(DRAWER_NS + '/status', String, self.on_status, queue_size=10)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        with self.lock:
            self.frame_id = msg.header.frame_id or 'odom'
            self.samples.append((msg.header.stamp.to_sec(), msg.pose.pose.position.x,
                                 msg.pose.pose.position.y, sg.yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def on_plan(self, msg):
        with self.lock:
            self.planned = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def on_status(self, msg):
        self.status = msg.data
        if msg.data == 'done' or msg.data.startswith('aborted'):
            self.final = msg.data

    def publish_path(self):
        with self.lock:
            tail = self.samples[-self.max_poses:]
            frame_id = self.frame_id
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        for t, x, y, _ in tail:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def spin(self):
        while not rospy.is_shutdown() and self.final is None:
            self.publish_path()
            time.sleep(PUBLISH_PERIOD)
        if self.final is not None:
            self.publish_path()
            self.write('status %s' % self.final)

    # -----------------------------------------------------
    # Output
    # -----------------------------------------------------
    def write(self, why):
        if self.written:
            return
        self.written = True
        with self.lock:
            samples = list(self.samples)
            planned = list(self.planned)
        if self.final == 'aborted: not confirmed':
            rospy.loginfo('path_recorder: run not confirmed, robot never moved; nothing written')
            return
        if not planned or not samples:
            rospy.loginfo('path_recorder: nothing to record (%s; %d planned, %d odom samples)',
                          why, len(planned), len(samples))
            return

        drawer = self.drawer_params
        if not rospy.is_shutdown():
            try:
                drawer = rospy.get_param(DRAWER_NS, {})
            except Exception:
                pass
        shape = drawer.get('shape', 'shape')
        stamp = time.strftime('%Y%m%d_%H%M%S')
        if not os.path.isdir(self.out_dir):
            os.makedirs(self.out_dir)
        base = os.path.join(self.out_dir, '%s_%s' % (stamp, shape))

        with open(base + '.csv', 'w') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw'])
            for row in samples:
                w.writerow(['%.3f' % row[0], '%.4f' % row[1], '%.4f' % row[2], '%.4f' % row[3]])

        m = self.metrics(samples, planned, drawer)
        self.plot(base + '.png', samples, planned, drawer, shape, m, why)
        rospy.loginfo('path_recorder: %s | %s: closure %.2f cm, cross-track max %.2f cm, mean %.2f cm '
                      '(%d samples) -> %s.{csv,png}',
                      why, shape, m['closure'] * 100, m['xt_max'] * 100, m['xt_mean'] * 100,
                      len(samples), base)

    @staticmethod
    def metrics(samples, planned, drawer):
        """Closure error and cross-track error over the part after the robot reached the shape start."""
        xy = [(s[1], s[2]) for s in samples]
        reach = drawer.get('stop_tol', 0.02) + 0.005
        first = next((i for i, p in enumerate(xy)
                      if sg.closure_error([p], planned[0]) <= reach), len(xy) - 1)
        xt = sg.cross_track_errors(xy[first:], planned)
        return {
            'closure': sg.closure_error(xy, planned[0]),
            'xt_max': max(xt),
            'xt_mean': sum(xt) / len(xt),
        }

    @staticmethod
    def to_area(points, drawer):
        """Odom points -> area frame using the drawer's odom0 and start (identity if unknown)."""
        odom0 = drawer.get('odom0')
        if not odom0 or 'start_x' not in drawer:
            return points
        start = (drawer['start_x'], drawer['start_y'])
        return [sg.odom_to_area(p[0], p[1], start, odom0) for p in points]

    def plot(self, png, samples, planned, drawer, shape, m, why):
        actual = self.to_area([(s[1], s[2]) for s in samples], drawer)
        plan_a = self.to_area(planned, drawer)
        fig, ax = plt.subplots(figsize=(6, 6), dpi=120)
        if 'area_w' in drawer and drawer.get('odom0'):
            W, H = drawer['area_w'], drawer['area_h']
            ub = sg.usable_box(W, H, drawer['margin'], drawer['footprint_r'])
            ax.add_patch(plt.Rectangle((0, 0), W, H, fill=False, ec=COLOR_BOX, lw=1.5, label='area'))
            ax.add_patch(plt.Rectangle((ub[0], ub[1]), ub[2] - ub[0], ub[3] - ub[1], fill=False,
                                       ec=COLOR_BOX, lw=1, ls=':', label='usable box (robot center)'))
            ax.set_xlim(-0.05 * W, 1.05 * W)
            ax.set_ylim(-0.05 * H, 1.05 * H)
            ax.plot(drawer['start_x'], drawer['start_y'], marker='o', ms=8, color=COLOR_BOX,
                    ls='none', label='robot start')
            frame = 'area frame'
        else:
            frame = 'odom frame'
        ax.plot([p[0] for p in plan_a], [p[1] for p in plan_a], ls='--', lw=2, color=COLOR_PLANNED,
                label='planned')
        ax.plot([p[0] for p in actual], [p[1] for p in actual], ls='-', lw=2, color=COLOR_ACTUAL,
                label='actual (odometry)')
        ax.set_aspect('equal')
        ax.set_xlabel('x (m, %s)' % frame)
        ax.set_ylabel('y (m)')
        ax.grid(True, color='#e4e3df', lw=0.6)
        ax.set_title('%s — %s\nclosure %.1f cm · cross-track max %.1f cm, mean %.1f cm'
                     % (shape, why, m['closure'] * 100, m['xt_max'] * 100, m['xt_mean'] * 100), fontsize=10)
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        fig.tight_layout()
        fig.savefig(png)
        plt.close(fig)


def main():
    rospy.init_node('path_recorder')
    rec = PathRecorder()
    rec.drawer_params = {}

    def cache_params():
        # Drawer params are gone once the master shuts down; keep a fresh copy
        while not rospy.is_shutdown():
            try:
                rec.drawer_params = rospy.get_param(DRAWER_NS, {})
            except Exception:
                pass
            time.sleep(0.5)

    threading.Thread(target=cache_params, daemon=True).start()
    rospy.on_shutdown(lambda: rec.write('shutdown (status %s)' % (rec.status or 'none')))
    rec.spin()


if __name__ == '__main__':
    main()
