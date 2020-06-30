#!/usr/bin/env python
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


def get_key():
    # tty.setraw changes file descriptor fd to raw, tty is used for terminal control functions
    tty.setraw(sys.stdin.fileno())
    # interface to unix select() system call
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    # Set the tty attributes for file descriptor fd from the attributes
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def talker():
    # define topic to publish on
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # register node at rosmaster
    rospy.init_node('keyboard_control_node')
    speed_increment = 0.05

    twist_msg = Twist()
    print("Navigate with keyboard. Hold to accelerate:")
    print("w = increase linear speed in x direction (forward)")
    print("s = decrease linear speed in x direction (forward)")
    print("d = increase linear speed in y direction (to the right)")
    print("a = decrease linear speed in y direction (to the right)")
    print("e = increase angular speed around z axis (turn right)")
    print("q = decrease angular speed around z axis (turn right)")
    print("any other key = stop")
    print("ESC = quit node")

    while not rospy.is_shutdown():
        pressed_key = get_key()
        # linear
        if pressed_key == 'w':
            twist_msg.linear.x += speed_increment
        elif pressed_key == 's':
            twist_msg.linear.x -= speed_increment
        elif pressed_key == 'a':
            twist_msg.linear.y += speed_increment
        elif pressed_key == 'd':
            twist_msg.linear.y -= speed_increment

        # angular
        elif pressed_key == 'e':
            twist_msg.angular.z -= speed_increment
        elif pressed_key == 'q':
            twist_msg.angular.z += speed_increment

        elif pressed_key == '\x1b' or pressed_key == '\x03':
            rospy.logwarn("Input was canceled. Node will quit ...")
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            break
        else:
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0

        pub.publish(twist_msg)


if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        talker()
    except rospy.ROSInterruptException:
        pass
