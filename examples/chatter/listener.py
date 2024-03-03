import example_msgs.msg
import rospy


def callback(msg):
    rospy.loginfo(msg)


def main():
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber(  # noqa
        'chatter', example_msgs.msg.Example, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
