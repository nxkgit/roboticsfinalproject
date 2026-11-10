#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-10-28
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Shape Drawing (Milestone 3)
#
# fake_odom_sim.py
# Stand-in for the robot base in simulation: integrates the body-frame
# Twist on /cmd_vel into a world pose and publishes nav_msgs/Odometry
# (frame odom -> base_footprint). A stale command (> ~cmd_timeout)
# counts as zero velocity, like a real base driver's watchdog.
#   ~linear_scale / ~angular_scale  model wheel-calibration error
#   ~noise_std  std (m) of Gaussian noise added to each step's x/y
#               displacement while moving
# ---------------------------------------------------------
import math
import random

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def integrate(x, y, yaw, vx, vy, wz, dt):
    """Advance a planar pose by a body-frame velocity for dt seconds (midpoint heading)."""
    mid = yaw + 0.5 * wz * dt
    c, s = math.cos(mid), math.sin(mid)
    x += (c * vx - s * vy) * dt
    y += (s * vx + c * vy) * dt
    yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
    return x, y, yaw


class FakeOdomSim(object):

    def __init__(self):
        self.rate = rospy.get_param('~rate', 50.0)
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)
        self.noise_std = rospy.get_param('~noise_std', 0.0)
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)
        self.x = rospy.get_param('~x0', 0.0)
        self.y = rospy.get_param('~y0', 0.0)
        self.yaw = rospy.get_param('~yaw0', 0.0)

        self.cmd = Twist()
        self.cmd_time = None
        self.pub = rospy.Publisher(rospy.get_param('~odom_topic', '/odom'), Odometry, queue_size=10)
        rospy.Subscriber(rospy.get_param('~cmd_vel_topic', '/cmd_vel'), Twist, self.on_cmd, queue_size=1)

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = rospy.Time.now()

    def velocity(self, now):
        """Body velocity actually applied: zero if the last command is stale."""
        if self.cmd_time is None or (now - self.cmd_time).to_sec() > self.cmd_timeout:
            return 0.0, 0.0, 0.0
        return (self.cmd.linear.x * self.linear_scale,
                self.cmd.linear.y * self.linear_scale,
                self.cmd.angular.z * self.angular_scale)

    def spin(self):
        rate = rospy.Rate(self.rate)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            last = now
            vx, vy, wz = self.velocity(now)
            self.x, self.y, self.yaw = integrate(self.x, self.y, self.yaw, vx, vy, wz, dt)
            if self.noise_std > 0 and (vx or vy):
                self.x += random.gauss(0.0, self.noise_std)
                self.y += random.gauss(0.0, self.noise_std)
            self.publish(now, vx, vy, wz)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def publish(self, now, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub.publish(odom)


def main():
    rospy.init_node('fake_odom_sim')
    FakeOdomSim().spin()


if __name__ == '__main__':
    main()
