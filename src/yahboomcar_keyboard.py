#!/usr/bin/env python3
# ---------------------------------------------------------
# Name:   <Your Name>
# Date:   2026-10-23
# Course: CSIS-370 Robotics, Dr. Liu
# Lecture 11 - RosMaster Navigation I
#
# yahboomcar_keyboard.py
# Keyboard teleop for the Yahboom ROSMASTER X3 Plus (mecanum wheels).
# Reads single keypresses in raw terminal mode and publishes
# geometry_msgs/Twist on /cmd_vel. Lowercase keys rotate (angular.z),
# capital / SHIFT keys strafe sideways (linear.y).
# ---------------------------------------------------------
import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

BANNER = """
---------------------------------------------------------
ROSMASTER X3 Plus Keyboard Teleop (Mecanum Holonomic Control)
---------------------------------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Standard Controls (Lowercase - Rotation / Non-Holonomic):
---------------------------------------------------------
  i : Forward                 , : Backward
  j : Rotate Left             l : Rotate Right
  u : Forward + Turn Left     o : Forward + Turn Right
  m : Backward + Turn Left    . : Backward + Turn Right
  k : Force Stop

Mecanum Controls (Capital / SHIFT - Sidewalk / Holonomic):
---------------------------------------------------------
  I : Forward                 < : Backward
  J : Strafe Left             L : Strafe Right
  U : Forward + Strafe Left   O : Forward + Strafe Right
  M : Backward + Strafe Left  > : Backward + Strafe Right
  K : Force Stop

Speed Controls:
---------------------------------------------------------
  q/z : increase/decrease max speeds by 10%
  w/x : increase/decrease linear speed only by 10%
  e/c : increase/decrease angular speed only by 10%

CTRL-C to quit
"""

# key -> (linear_x_factor, linear_y_factor, angular_z_factor)
MOVE_BINDINGS = {
    # --- Standard Rotation / Non-Holonomic Control (Lowercase) ---
    'i': ( 1.0,  0.0,  0.0),   # Forward
    ',': (-1.0,  0.0,  0.0),   # Backward
    'j': ( 0.0,  0.0,  1.0),   # Rotate Left (Yaw)
    'l': ( 0.0,  0.0, -1.0),   # Rotate Right (Yaw)
    'u': ( 1.0,  0.0,  1.0),   # Forward + Rotate Left
    'o': ( 1.0,  0.0, -1.0),   # Forward + Rotate Right
    'm': (-1.0,  0.0, -1.0),   # Backward + Rotate Left
    '.': (-1.0,  0.0,  1.0),   # Backward + Rotate Right
    'k': ( 0.0,  0.0,  0.0),   # Force Stop

    # --- Mecanum Holonomic / Strafe Control (Capitalized / Shift Keys) ---
    'I': ( 1.0,  0.0,  0.0),   # Pure Forward
    '<': (-1.0,  0.0,  0.0),   # Pure Backward
    'J': ( 0.0,  1.0,  0.0),   # Pure Side Walk Left
    'L': ( 0.0, -1.0,  0.0),   # Pure Side Walk Right
    'U': ( 1.0,  1.0,  0.0),   # Forward + Side Walk Left
    'O': ( 1.0, -1.0,  0.0),   # Forward + Side Walk Right
    'M': (-1.0,  1.0,  0.0),   # Backward + Side Walk Left
    '>': (-1.0, -1.0,  0.0),   # Backward + Side Walk Right
    'K': ( 0.0,  0.0,  0.0),   # Force Stop
}

# key -> (linear_mult, angular_mult)
SPEED_BINDINGS = {
    'q': (1.1, 1.1),   # both up 10%
    'z': (0.9, 0.9),   # both down 10%
    'w': (1.1, 1.0),   # linear up 10%
    'x': (0.9, 1.0),   # linear down 10%
    'e': (1.0, 1.1),   # angular up 10%
    'c': (1.0, 0.9),   # angular down 10%
}


def get_key(settings, timeout=0.1):
    """Read one keypress in raw mode; return '' if none arrives within timeout."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_twist(x, y, th, linear_speed, angular_speed):
    """Build a Twist from direction factors and the current speed limits."""
    twist = Twist()
    twist.linear.x = x * linear_speed
    twist.linear.y = y * linear_speed   # NEW: strafing uses the linear speed
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = th * angular_speed
    return twist


def speeds_str(linear_speed, angular_speed):
    return "currently:\tlinear %.3f m/s\tangular %.3f rad/s" % (linear_speed, angular_speed)


def main():
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('yahboomcar_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Safe defaults for the real robot; override with _speed:=... _turn:=...
    linear_speed = rospy.get_param('~speed', 0.2)
    angular_speed = rospy.get_param('~turn', 1.0)

    x = y = th = 0.0
    try:
        print(BANNER)
        print(speeds_str(linear_speed, angular_speed))
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key in MOVE_BINDINGS:
                x, y, th = MOVE_BINDINGS[key]
            elif key in SPEED_BINDINGS:
                linear_speed *= SPEED_BINDINGS[key][0]
                angular_speed *= SPEED_BINDINGS[key][1]
                print(speeds_str(linear_speed, angular_speed))
            else:
                # Unbound key or no key (released): stop the robot
                x = y = th = 0.0
                if key == '\x03':   # CTRL-C
                    break
            pub.publish(make_twist(x, y, th, linear_speed, angular_speed))
    except Exception as e:
        print(e)
    finally:
        # Always leave the robot stopped and the terminal usable
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
