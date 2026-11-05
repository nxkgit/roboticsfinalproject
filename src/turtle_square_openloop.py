#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   Michael Leonhard
# Date:   2026-10-28
# Course: CSIS-370 Robotics, Dr. Liu
# Final Project - Shape Drawing (Milestone 2)
#
# turtle_square_openloop.py
# Timed open-loop square: each move runs for distance / speed seconds,
# with a zero Twist between moves and in `finally`.
#   _mode:=turn    drive forward, turn 90 deg in place (non-holonomic, turtlesim)
#   _mode:=strafe  +x, +y, -x, -y at constant heading (mecanum)
# ---------------------------------------------------------
import math

import rospy
from geometry_msgs.msg import Twist

# move name -> (linear_x_factor, linear_y_factor, angular_z_factor)
MOVE_BINDINGS = {
    'forward': ( 1.0,  0.0,  0.0),
    'left':    ( 0.0,  1.0,  0.0),   # strafe left
    'back':    (-1.0,  0.0,  0.0),
    'right':   ( 0.0, -1.0,  0.0),   # strafe right
    'turn':    ( 0.0,  0.0,  1.0),   # rotate left (CCW)
}

# Square as a sequence of moves; translations cover `side`, 'turn' covers 90 deg
SQUARE_MOVES = {
    'turn':   ['forward', 'turn'] * 4,
    'strafe': ['forward', 'left', 'back', 'right'],
}

# Hard caps used whenever the topic is not a turtlesim topic (i.e. a real base)
MAX_LINEAR = 0.30
MAX_ANGULAR = 1.0


def make_twist(x, y, th, linear_speed, angular_speed):
    """Build a Twist from direction factors and the current speed limits."""
    twist = Twist()
    twist.linear.x = x * linear_speed
    twist.linear.y = y * linear_speed
    twist.angular.z = th * angular_speed
    return twist


def move_duration(move, side, linear_speed, angular_speed):
    """Seconds to run a move: side / speed for translations, (pi/2) / turn for a turn."""
    if move == 'turn':
        return (math.pi / 2.0) / angular_speed
    return side / linear_speed


def run_move(pub, twist, duration, rate):
    """Publish twist for duration seconds, then a zero Twist."""
    end = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())
    rospy.sleep(0.3)


def main():
    rospy.init_node('turtle_square_openloop')

    side = rospy.get_param('~side', 2.0)
    linear_speed = rospy.get_param('~speed', 0.5)
    angular_speed = rospy.get_param('~turn', 1.0)
    topic = rospy.get_param('~cmd_vel_topic', '/turtle1/cmd_vel')
    mode = rospy.get_param('~mode', 'turn')
    if mode not in SQUARE_MOVES:
        rospy.logerr('~mode must be turn or strafe, got %r', mode)
        return
    if not topic.startswith('/turtle'):
        linear_speed = min(linear_speed, MAX_LINEAR)
        angular_speed = min(angular_speed, MAX_ANGULAR)

    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rate = rospy.Rate(20)

    try:
        # Give subscribers a moment to connect so the first move is not lost
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while pub.get_num_connections() == 0 and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo('open-loop square: mode=%s side=%.2f m speed=%.2f m/s turn=%.2f rad/s on %s',
                      mode, side, linear_speed, angular_speed, topic)
        for move in SQUARE_MOVES[mode]:
            if rospy.is_shutdown():
                break
            x, y, th = MOVE_BINDINGS[move]
            duration = move_duration(move, side, linear_speed, angular_speed)
            rospy.loginfo('%-7s for %.2f s', move, duration)
            run_move(pub, make_twist(x, y, th, linear_speed, angular_speed), duration, rate)
    finally:
        # Always leave the robot stopped
        pub.publish(Twist())


if __name__ == '__main__':
    main()
