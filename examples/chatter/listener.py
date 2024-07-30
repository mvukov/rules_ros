#!/usr/bin/env python3
from examples.chatter.example_msgs.msg import Example
import rospy


def callback(msg):
    rospy.loginfo(msg)


def main():
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber(  # noqa
        'chatter', Example, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
