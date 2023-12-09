#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

key_mapping = {
    'w': (1, 0, 0, 0, 0, 0),
    's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0),
    'd': (0, -1, 0, 0, 0, 0),
    '1': (0, 0, 0, 1, 0, 0),
    '3': (0, 0, 0, -1, 0, 0),
    '8': (0, 0, 1, 0, 0, 0),
    '5': (0, 0, -1, 0, 0, 0),
    '4': (0, 0, 0, 0, 1, 0),
    '6': (0, 0, 0, 0, -1, 0),
    '7': (0, 0, 0, 0, 0, 1),
    '9': (0, 0, 0, 0, 0, -1),

    
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop():
    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/polydog_cmd', Twist, queue_size=10)
    twist = Twist()

    try:
        print("Use 'WASDQE' to control the robot. Press 'Ctrl + C' to exit.")
        while not rospy.is_shutdown():
            key = getKey()
            if key in key_mapping:
                twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z, twist.angular.x, twist.angular.y= key_mapping[key]
                pub.publish(twist)
            else:
                twist = Twist()
                pub.publish(twist)

    except KeyboardInterrupt:
        print("Teleop interrupted by user.")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

